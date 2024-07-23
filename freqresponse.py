import instruments
import matplotlib.pyplot as plt
import argparse
import math
import time
import sys

def main():
    #parse command line arguments
    parser = argparse.ArgumentParser(description='Measure audio-spectrum frequency response.')
    parser.add_argument('-dmm', type=str, help='DMM port (default: Auto)')
    parser.add_argument('-gen', type=str, help='Signal generator port (default: Auto)')
    parser.add_argument('-scope', type=str, help='Oscilloscope port (default: Auto)')
    parser.add_argument('-s', '--steps', type=int, default=3, help='Number of steps per octave (default: 3)')
    parser.add_argument('-v', '--voltage', type=float, default=3.0, help='Signal generator RMS voltage (default: 3.0Vrms)')
    parser.add_argument('-r', '--reference', type=float, help='0dB reference Vrms (default: Auto)')
    parser.add_argument('-phase', action='store_true', help='Gather phase data, CH1->CH2')
    parser.add_argument('-p', '--plot', type=str, nargs='?', const='NO_OUTPUT_PLOT_IMAGE_FILE', help='Plot data [filename]')
    parser.add_argument('-n', '--name', type=str, help='Dataset name')
    args = parser.parse_args()

    voltage = args.voltage
    if voltage > 5.0:
        voltage = 5.0
    # calculate 0dB reference if applicable
    if args.reference is not None:
        reference = args.reference
    else:
        reference = args.voltage

    #initialize data
    data = [0, 0, 0, 0]
    #initalize instruments
    dmm = instruments.HP_34401A()
    arb = instruments.Siglent_SDG1032X()
    try:
        dmm.open()
        print('Connected to DMM at {}'.format(dmm.port))
    except OSError:
        print('Could not connect to DMM')
        sys.exit()
    try:
        arb.open()
        print('Connected to signal generator at {}'.format(arb.port))
    except OSError:
        print('Could not connect to signal generator')
        sys.exit()
    if args.phase is True:
        try:
            scope = instruments.Rigol_DS1104Z()
            scope.open()
            print('Connected to oscilloscope at {}'.format(scope.port))
        except OSError:
            print('Could not connect to oscilloscope')
            sys.exit()

    arb.output(1, False)
    arb.amplitude(1, voltage, rms=True)
    arb.waveform(1, 'SINE')
    print('{} Vrms sine wave signal, {} Vrms 0dB reference'.format(voltage, reference))

    freq = 10
    plot_freq = []
    plot_db = []
    if args.phase is True:
        plot_phase = []
        scope.timebase_scale(1/(2*freq))
        scope.channel_scale(1, voltage)
    #enable signal output
    arb.output(1, True)

    while freq < 25000:
        #set frequency
        arb.frequency(1, freq)
        data[0] = round(freq, 2)
        #measure voltage
        result = dmm.volts_ac('DEF', 'MIN')
        data[1] = result
        #calculate gain
        db = 20*math.log10(result/reference)
        data[2] = db
        #compile data
        plot_freq.append(freq)
        plot_db.append(db)

        if args.phase is True:
            #take phase measurement
            scope.timebase_scale(1 / (2 * freq))
            scope.channel_scale(2, result)
            scope.run()
            time.sleep(10/freq)
            scope.trigger_sweep('SING')
            scope.acquire_memory_depth(600000)
            scope.trigger_force()
            time.sleep(20/freq)
            scope.stop()
            phase = scope.phase_offset(1, 2, freq)
            data[3] = phase
            print('{} Hz: {} Vrms | Gain: {} dB | Phase: {}°'.format(
                round(freq, 2), result, round(db, 2), round(phase, 2)))
            plot_phase.append(phase)
        else:
            print('{} Hz: {} Vrms | Gain: {} dB'.format(round(freq, 2), result, round(db, 2)))

        #generate next frequency
        freq = freq * (2 ** (1 / args.steps))

    arb.output(1, False)

    #generate Bode plot
    fig, ax1 = plt.subplots()
    #plot amplitude
    ax1.set_xlabel('Frequency (Hz)')
    ax1.set_ylabel('Gain (dB)', color='tab:blue')
    ax1.set_xlim(10, 30000)
    ax1.set_xscale('log')
    ax1.plot(plot_freq, plot_db, color='tab:blue')
    for tl in ax1.get_yticklabels():
        tl.set_color('tab:blue')
    if args.phase is True:
        ax2 = ax1.twinx()
        ax2.set_ylabel('Phase (degrees)', color='tab:red')
        ax2.set_ylim(-95, 5)
        ax2.plot(plot_freq, plot_phase, color='tab:red')
        phase_ticks = [-90, -45, 0]
        ax2.set_yticks(phase_ticks)
        ax2.set_yticklabels(phase_ticks)
        for tl in ax2.get_yticklabels():
            tl.set_color('r')

    fig.tight_layout()

    if args.name is not None:
        fig.suptitle(args.name)
    if args.plot is not None:
        if args.plot == 'NO_OUTPUT_PLOT_IMAGE_FILE':
            pass
        elif args.plot[-4:] != '.png':
            plot_filename = args.plot + '.png'
            fig.savefig(plot_filename)
            print('Plot saved to {}'.format(plot_filename))
        else:
            plot_filename = args.plot
            fig.savefig(plot_filename)
            print('Plot saved to {}'.format(plot_filename))

    plt.show()

if __name__ == "__main__":
    main()
