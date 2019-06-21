#!/usr/bin/env python
#
# Copyright (C) 2019 Analog Devices, Inc.
# Author: Travis Collins <travis.collins@analog.com>
#
# Licensed under the GPL-2.

import sys
try:
    import iio
except:
    # By default the iio python bindings are not in path
    sys.path.append('/usr/lib/python2.7/site-packages/')
    import iio

import numpy as np
try:
    import matplotlib.pyplot as plt
    do_plots = True
except:
    print("To view plots install matplotlib")
    do_plots = False

def measure_phase(chan0,chan1):
    errorV = np.angle(chan0 * np.conj(chan1))*180/np.pi
    error = np.mean(errorV)
    return error

def run_hardware_tests(TRXLO,sync,runs,uri):

    # User configurable
    DDS_Freq = 4000000

    # Setup contexts
    try:
        ctx = iio.Context(uri)
    except:
        raise Exception("No device found")

    ctrl_chipA = ctx.find_device("adrv9009-phy")
    ctrl_chipB = ctx.find_device("adrv9009-phy-b")
    txdac = ctx.find_device("axi-adrv9009-tx-hpc")
    rxadc = ctx.find_device("axi-adrv9009-rx-hpc")
    hmc7044 = ctx.find_device("hmc7044")

    # Configure transceiver settings
    LO = ctrl_chipA.find_channel("TRX_LO", True)
    LO.attrs["frequency"].value = str(int(TRXLO))
    LO = ctrl_chipB.find_channel("TRX_LO", True)
    LO.attrs["frequency"].value = str(int(TRXLO))
    #
    rx = ctrl_chipA.find_channel("voltage0")
    rx.attrs['gain_control_mode'].value = 'slow_attack'
    rx = ctrl_chipB.find_channel("voltage0")
    rx.attrs['gain_control_mode'].value = 'slow_attack'

    if sync:
        # Calibrate
        ctrl_chipA.attrs['calibrate_rx_phase_correction_en'] = True
        ctrl_chipA.attrs['calibrate'] = True
        ctrl_chipB.attrs['calibrate_rx_phase_correction_en'] = True
        ctrl_chipB.attrs['calibrate'] = True

        # MCS
        hmc7044.reg_write(0x5a,0)
        for k in range(12):
            ctrl_chipA.attrs['multichip_sync'] = str(k)
            ctrl_chipB.attrs['multichip_sync'] = str(k)

    # Enable all IQ channels
    v0 = rxadc.find_channel("voltage0_i")
    v1 = rxadc.find_channel("voltage0_q")
    v2 = rxadc.find_channel("voltage1_i")
    v3 = rxadc.find_channel("voltage1_q")
    v0.enabled = True
    v1.enabled = True
    v2.enabled = True
    v3.enabled = True


    # Create buffer for RX data
    rxbuf = iio.Buffer(rxadc, 2**14, False)
    #
    # Enable single tone DDS
    dds0_tx1 = txdac.find_channel('altvoltage0',True)
    dds2_tx1 = txdac.find_channel('altvoltage2',True)
    dds0_tx2 = txdac.find_channel('altvoltage8',True)
    dds2_tx2 = txdac.find_channel('altvoltage10',True)

    # Turn all others off
    dds1 = txdac.find_channel('altvoltage1',True)
    dds1.attrs['raw'].value = str(0)
    dds1.attrs['scale'].value = str(0)
    for r in range(3,16):
        dds1 = txdac.find_channel('altvoltage'+str(r),True)
        dds1.attrs['scale'].value = str(0)
        dds1.attrs['raw'].value = str(0)

    # Set frequency of enabled DDSs
    dds0_tx1.attrs['raw'].value = str(1)
    dds0_tx1.attrs['frequency'].value = str(DDS_Freq)
    dds0_tx1.attrs['scale'].value = str(0.5)
    dds0_tx1.attrs['phase'].value = str(90000)
    dds2_tx1.attrs['raw'].value = str(1)
    dds2_tx1.attrs['frequency'].value = str(DDS_Freq)
    dds2_tx1.attrs['scale'].value = str(0.5)
    dds2_tx1.attrs['phase'].value = str(0)

    dds0_tx2.attrs['raw'].value = str(1)
    dds0_tx2.attrs['frequency'].value = str(DDS_Freq)
    dds0_tx2.attrs['scale'].value = str(0.5)
    dds0_tx2.attrs['phase'].value = str(90000)
    dds2_tx2.attrs['raw'].value = str(1)
    dds2_tx2.attrs['frequency'].value = str(DDS_Freq)
    dds2_tx2.attrs['scale'].value = str(0.5)
    dds2_tx2.attrs['phase'].value = str(0)


    # Collect data
    # reals0 = np.array([])
    # imags0 = np.array([])
    # reals1 = np.array([])
    # imags1 = np.array([])
    phase_error = np.array([])

    for i in range(runs):
      rxbuf.refill()
      data = rxbuf.read()
      x = np.frombuffer(data,dtype=np.int16)
      # reals0 = np.append(reals0,x[::4])
      # imags0 = np.append(imags0,x[1::4])
      # reals1 = np.append(reals1,x[2::4])
      # imags1 = np.append(imags1,x[3::4])
      chan0 = x[0::4] + 1j*x[1::4]
      chan1 = x[2::4] + 1j*x[3::4]
      phase_error = np.append(phase_error,measure_phase(chan0,chan1))

    # # Plot
    # if do_plots:
    #     plt.plot(phase_error)
    #     # plt.plot(reals0)
    #     # # plt.plot(imags0)
    #     # plt.plot(reals1)
    #     # # plt.plot(imags1)
    #     plt.xlabel("Samples")
    #     plt.ylabel("Amplitude [dbFS]")
    #     plt.show()
    return phase_error
