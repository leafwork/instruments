import asyncio
import pyvisa
from pyvisa import constants
from math import sqrt
import numpy
from scipy import signal
import re
import minimalmodbus


class ModbusInstrument:
    def __init__(self):
        self.port = None
        self.address = None
        self.instrument = None

    def __del__(self):
        self.instrument.serial.close()

    def read_register(self, register):
        try:
            return self.instrument.read_register(register)
        except minimalmodbus.NoResponseError:
            raise OSError('Communication error')

    def write_register(self, register, value):
        try:
            self.instrument.write_register(register, value)
        except minimalmodbus.NoResponseError:
            raise OSError('Communication error')

    def identify(self):
        return self.read_register(0)


class ScpiInstrument:
    def __init__(self):
        self.resourceManager = pyvisa.ResourceManager()
        self.port = None
        self.instrument = None

    def __del__(self):
        self.instrument.close()

    def write(self, message):
        try:
            self.instrument.write(message)
        except pyvisa.errors.VisaIOError:
            raise OSError('Communication error')

    def query(self, message):
        try:
            return self.instrument.query(message).rstrip()
        except pyvisa.errors.VisaIOError:
            raise OSError('Communication error')

    def identify(self):
        return self.query('*IDN?')

    def reset(self):
        self.write('*RST')

    def complete(self):
        return self.query('*OPC?')

    def wait(self):
        self.write('*WAI')


class Riden_RD6006(ModbusInstrument):
    def open(self, port=None):
        if port is not None:
            self.port = port
        else:
            self.port = 'COM11'
            self.address = 1
            self.instrument = minimalmodbus.Instrument(port=self.port, slaveaddress=self.address)
            self.instrument.serial.baudrate = 115200
            self.instrument.serial.timeout = 1

    def voltage_setting(self, voltage=None):
        if voltage is not None:
            self.write_register(8, voltage * 100)
        else:
            return float(self.read_register(8)) / 100

    def current_setting(self, current=None):
        if current is not None:
            self.write_register(9, current * 1000)
        else:
            return float(self.read_register(9)) / 1000

    def voltage_display(self):
        return float(self.read_register(10)) / 100

    def current_display(self):
        return float(self.read_register(11)) / 1000

    def power_display(self):
        return float(self.read_register(13)) / 100

    def output(self, output_enable=None):
        if output_enable is not None:
            if output_enable is True:
                self.write_register(18, 1)
            else:
                self.write_register(18, 0)
        else:
            return bool(self.read_register(18))

    def mode(self):
        current_mode = int(self.read_register(17))
        if current_mode == 0:
            return 'CV'
        elif current_mode == 1:
            return 'CC'


class HP_34401A(ScpiInstrument):
    def open(self, port=None):
        if port is not None:
            self.port = port
        else:
            self.port = 'ASRL1::INSTR'
        try:
            self.instrument = self.resourceManager.open_resource(self.port)
            self.instrument.timeout = 10000
            self.instrument.set_visa_attribute(constants.VI_ATTR_ASRL_FLOW_CNTRL, constants.VI_ASRL_FLOW_DTR_DSR)
            self.write('syst:rem')
            self.identify()
            self.lock = asyncio.Lock()
        except OSError:
            raise OSError('Could not communicate with 34401A at {}'.format(self.port))

    def configure_volts_dc(self, set_range='DEF', resolution='DEF'):
        self.write(':CONFigure:VOLTage:DC {}, {}'.format(set_range, resolution))

    def volts_dc(self, set_range='DEF', resolution='DEF'):
        return float(self.query(':MEASure:VOLTage:DC? {}, {}'.format(set_range, resolution)).rstrip())

    def configure_volts_dc_ratio(self, set_range='DEF', resolution='DEF'):
        self.write(':CONFigure:VOLTage:DC:RATio {}, {}'.format(set_range, resolution))

    def volts_dc_ratio(self, set_range='DEF', resolution='DEF'):
        return float(self.query(':MEASure:VOLTage:DC:RATio? {}, {}'.format(set_range, resolution)).rstrip())

    def configure_volts_ac(self, set_range='DEF', resolution='DEF'):
        self.write(':CONFigure:VOLTage:AC {}, {}'.format(set_range, resolution))

    def volts_ac(self, set_range='DEF', resolution='DEF'):
        return float(self.query(':MEASure:VOLTage:AC? {}, {}'.format(set_range, resolution)).rstrip())

    def configure_current_dc(self, set_range='DEF', resolution='DEF'):
        self.write(':CONFigure:CURRent:DC {}, {}'.format(set_range, resolution))

    def current_dc(self, set_range='DEF', resolution='DEF'):
        return float(self.query(':MEASure:CURRent:DC? {}, {}'.format(set_range, resolution)).rstrip())

    def configure_current_ac(self, set_range='DEF', resolution='DEF'):
        self.write(':CONFigure:CURRent:AC {}, {}'.format(set_range, resolution))

    def current_ac(self, set_range='DEF', resolution='DEF'):
        return float(self.query(':MEASure:CURRent:AC? {}, {}'.format(set_range, resolution)).rstrip())

    def configure_resistance(self, set_range='DEF', resolution='DEF'):
        self.write(':CONFigure:RESistance {}, {}'.format(set_range, resolution))

    def resistance(self, set_range='DEF', resolution='DEF'):
        return float(self.query(':MEASure:RESistance? {}, {}'.format(set_range, resolution)).rstrip())

    def configure_resistance_4w(self, set_range='DEF', resolution='DEF'):
        self.write(':CONFigure:FRESistance {}, {}'.format(set_range, resolution))

    def resistance_4w(self, set_range='DEF', resolution='DEF'):
        return float(self.query(':MEASure:FRESistance? {}, {}'.format(set_range, resolution)).rstrip())

    def configure_frequency(self, set_range='DEF', resolution='DEF'):
        self.write(':CONFigure:FREQuency {}, {}'.format(set_range, resolution))

    def frequency(self, set_range='DEF', resolution='DEF'):
        return float(self.query(':MEASure:FREQuency? {}, {}'.format(set_range, resolution)).rstrip())

    def configure_period(self, set_range='DEF', resolution='DEF'):
        self.write(':CONFigure:PERiod {}, {}'.format(set_range, resolution))

    def period(self, set_range='DEF', resolution='DEF'):
        return float(self.query(':MEASure:PERiod? {}, {}'.format(set_range, resolution)).rstrip())


class Siglent_SDG1032X(ScpiInstrument):
    def open(self, port=None):
        if port is not None:
            self.port = port
        else:
            resources = self.resourceManager.list_resources()
            matching_resources = []
            for device in resources:
                if 'SDG1' in device:
                    matching_resources.append(device)
            if matching_resources:
                self.port = matching_resources[0]
            else:
                raise OSError('Matching device not found')
        try:
            self.instrument = self.resourceManager.open_resource(self.port)
            self.instrument.timeout = 10000
        except pyvisa.errors.VisaIOError:
            raise OSError('Could not connect to SDG1032X at {}'.format(self.port))

    def parse_settings(self, channel, search, type='basic'):
        if type == 'basic':
            settings = self.query('C{}:BSWV?'.format(channel))
        elif type == 'modulation':
            settings = self.query('C{}:MDWV?'.format(channel))
        elif type == 'sweep':
            settings = self.query('C{}:SWWV?'.format(channel))
        elif type == 'burst':
            settings = self.query('C{}:BTWV?'.format(channel))
        else:
            raise Exception('Incorrect settings parse type')
        search_result = re.search(search, settings)
        search_result = search_result.group(1)
        return search_result

    def output(self, channel, enabled=None):
        if enabled is not None:
            if enabled is True:
                self.write('C{}:OUTP ON'.format(channel))
            elif enabled is False:
                self.write('C{}:OUTP OFF'.format(channel))
        else:
            channel_settings = self.query('C{}:OUTP?'.format(channel))
            output_setting = re.search('OUTP (.*?),', channel_settings)
            output_setting = output_setting.group(1)
            if output_setting == 'ON':
                return True
            if output_setting == 'OFF':
                return False

    def frequency(self, channel, frequency=None):
        if frequency is not None:
            self.write('C{}:BSWV FRQ,{}'.format(channel, frequency))
        else:
            return float(self.parse_settings(channel, 'FRQ,(.*?)HZ'))

    def period(self, channel, period=None):
        if period is not None:
            self.write('C{}:BSWV PERI,{}'.format(channel, period))
        else:
            return float(self.parse_settings(channel, 'PERI,(.*?)S,'))

    def amplitude(self, channel, voltage=None, rms=False):
        if voltage is not None:
            if rms is True:
                voltage = voltage * (2 * sqrt(2))
            self.write('C{}:BSWV AMP,{}'.format(channel, voltage))
        else:
            return float(self.parse_settings(channel, 'AMP,(.*?)V,'))

    def waveform(self, channel, waveform=None):
        if waveform is not None:
            self.write('C{}:BSWV WVTP,{}'.format(channel, waveform))
        else:
            return self.parse_settings(channel, 'WVTP,(.*?),')

    def phase(self, channel, phase_offset=None):
        if phase_offset is not None:
            self.write('C{}:BSWV PHSE,{}'.format(channel, phase_offset))
        else:
            return float(self.parse_settings(channel, 'PHSE,(.*?)$'))

    def offset(self, channel, offset=None):
        if offset is not None:
            self.write('C{}:BSWV OFST,{}'.format(channel, offset))
        else:
            return float(self.parse_settings(channel, 'OFST,(.*?)V,'))

    def ramp_symmetry(self, channel, symmetry=None):
        self.waveform(channel, 'RAMP')
        if symmetry is not None:
            self.write('C{}:BSWV SYM,{}'.format(channel, symmetry))
        else:
            return float(self.parse_settings(channel, 'SYM,(.*?)$'))

    def noise_stdev(self, channel, stdev=None):
        self.waveform(channel, 'NOISE')
        if stdev is not None:
            self.write('C{}:BSWV STDEV,{}'.format(channel, stdev))
        else:
            return float(self.parse_settings(channel, 'STDEV,(.*?)V'))

    def noise_mean(self, channel, mean=None):
        self.waveform(channel, 'NOISE')
        if mean is not None:
            self.write('C{}:BSWV MEAN,{}'.format(channel, mean))
        else:
            return float(self.parse_settings(channel, 'MEAN,(.*?)V'))

    def pulse_width(self, channel, width=None):
        self.waveform(channel, 'PULSE')
        if width is not None:
            self.write('C{}:BSWV WIDTH,{}'.format(channel, width))
        else:
            return float(self.parse_settings(channel, 'WIDTH,(.*?),'))

    def pulse_rise(self, channel, rise=None):
        self.waveform(channel, 'PULSE')
        if rise is not None:
            self.write('C{}:BSWV RISE,{}'.format(channel, rise))
        else:
            return float(self.parse_settings(channel, 'RISE,(.*?)S'))

    def pulse_fall(self, channel, fall=None):
        self.waveform(channel, 'PULSE')
        if fall is not None:
            self.write('C{}:BSWV FALL,{}'.format(channel, fall))
        else:
            return float(self.parse_settings(channel, 'FALL,(.*?)S'))

    def pulse_delay(self, channel, delay=None):
        self.waveform(channel, 'PULSE')
        if delay is not None:
            self.write('C{}:BSWV DLY,{}'.format(channel, delay))
        else:
            return float(self.parse_settings(channel, 'DLY,(.*?)$'))

    def modulation_state(self, channel, state=None):
        if state is not None:
            if state is True:
                self.write('C{}:MDWV STATE,ON'.format(channel))
            else:
                self.write('C{}:MDWV STATE,OFF'.format(channel))
        else:
            modulation_state = self.parse_settings(channel, 'STATE,(.*?)$', type='modulation')
            if modulation_state == "OFF":
                return False
            else:
                return True

    def modulation_source(self, channel, source=None):
        self.modulation_state(channel, True)
        modulation_type = self.modulation_type(channel)
        if source is not None:
            self.write('C{}:MDWV {},SRC,{}'.format(channel, modulation_type, source))
        else:
            return self.parse_settings(channel, 'SRC,(.*?),', type='modulation')

    def modulation_type(self, channel, mtype=None):
        self.modulation_state(channel, True)
        if mtype is not None:
            self.write('C{}:MDWV {}'.format(channel, mtype))
        else:
            return self.parse_settings(channel, 'ON,(.*?),', type='modulation')

    def modulation_carrier_waveform(self, channel, waveform=None):
        self.modulation_state(channel, True)
        if waveform is not None:
            self.write('C{}:MDWV CARR,WVTP,{}'.format(channel, waveform))
        else:
            return self.parse_settings(channel, 'CARR,WVTP,(.*?),', type='modulation')

    def modulation_carrier_frequency(self, channel, frequency=None):
        self.modulation_state(channel, True)
        if frequency is not None:
            self.write('C{}:MDWV CARR,FRQ,{}'.format(channel, frequency))
        else:
            modulation_carrier_waveform = self.modulation_carrier_waveform(channel)
            return float(self.parse_settings(channel,
                                    'CARR,WVTP,{},FRQ,(.*?)HZ,'.format(modulation_carrier_waveform), type='modulation'))

    def modulation_carrier_phase(self, channel, phase=None):
        self.modulation_state(channel, True)
        if phase is not None:
            self.write('C{}:MDWV CARR,PHSE,{}'.format(channel, phase))
        else:
            modulation_carrier_waveform = self.modulation_carrier_waveform(channel)
            if modulation_carrier_waveform == 'RAMP' \
                    or modulation_carrier_waveform == 'SQUARE' \
                    or modulation_carrier_waveform == 'PULSE':
                modulation_carrier_phase = float(self.parse_settings(channel, 'PHSE,(.*?),', type='modulation'))
            else:
                modulation_carrier_phase = float(self.parse_settings(channel, 'PHSE,(.*?)$', type='modulation'))
            return modulation_carrier_phase

    def modulation_carrier_offset(self, channel, offset=None):
        self.modulation_state(channel, True)
        if offset is not None:
            self.write('C{}:MDWV CARR,OFST,{}'.format(channel, offset))
        else:
            modulation_carrier_waveform = self.modulation_carrier_waveform(channel)
            if modulation_carrier_waveform == 'RAMP' \
                    or modulation_carrier_waveform == 'SQUARE' \
                    or modulation_carrier_waveform == 'PULSE':
                modulation_carrier_offset = float(self.parse_settings(channel, 'OFST,(.*?)V,', type='modulation'))
            else:
                modulation_carrier_offset = float(self.parse_settings(channel, 'OFST,(.*?)V$', type='modulation'))
            return modulation_carrier_offset

    def modulation_carrier_symmetry(self, channel, symmetry=None):
        self.modulation_state(channel, True)
        self.modulation_carrier_waveform(channel, 'RAMP')
        if symmetry is not None:
            self.write('C{}:MDWV CARR,SYM,{}'.format(channel, symmetry))
        else:
            return float(self.parse_settings(channel, 'SYM,(.*?)$', type='modulation'))

    def modulation_carrier_duty(self, channel, duty=None):
        self.modulation_state(channel, True)
        self.modulation_carrier_waveform(channel, 'SQUARE')
        if duty is not None:
            self.write('C{}:MDWV CARR,DUTY,{}'.format(channel, duty))
        else:
            return float(self.parse_settings(channel, 'DUTY,(.*?)$', type='modulation'))

    def modulation_carrier_rise(self, channel, rise=None):
        self.modulation_state(channel, True)
        self.modulation_carrier_waveform(channel, 'PULSE')
        if rise is not None:
            self.write('C{}:MDWV CARR,RISE,{}'.format(channel, rise))
        else:
            return float(self.parse_settings(channel, 'RISE,(.*?)S', type='modulation'))

    def modulation_carrier_fall(self, channel, fall=None):
        self.modulation_state(channel, True)
        self.modulation_carrier_waveform(channel, 'PULSE')
        if fall is not None:
            self.write('C{}:MDWV CARR,FALL,{}'.format(channel, fall))
        else:
            return float(self.parse_settings(channel, 'FALL,(.*?)S', type='modulation'))

    def modulation_shape(self, channel, shape=None):
        self.modulation_state(channel, True)
        modulation_type = self.modulation_type(channel)
        if shape is not None:
            self.modulation_source(channel, 'INT')
            self.write('C{}:MDWV {},MDSP,{}'.format(channel, modulation_type, shape))
        else:
            return self.parse_settings(channel, 'MDSP,(.*?),', type='modulation')

    def modulation_am_depth(self, channel, depth=None):
        self.modulation_state(channel, True)
        self.modulation_type(channel, 'AM')
        if depth is not None:
            self.modulation_source(channel, 'INT')
            self.write('C{}:MDWV AM,DEPTH,{}'.format(channel, depth))
        else:
            return float(self.parse_settings(channel, 'DEPTH,(.*?),', type='modulation'))

    def modulation_am_frequency(self, channel, frequency=None):
        self.modulation_state(channel, True)
        self.modulation_type(channel, 'AM')
        if frequency is not None:
            self.modulation_source(channel, 'INT')
            self.write('C{}:MDWV AM,FRQ,{}'.format(channel, frequency))
        else:
            return float(self.parse_settings(channel, 'FRQ,(.*?)HZ', type='modulation'))


class Rigol_DS1104Z(ScpiInstrument):
    def open(self, port=None):
        if port is not None:
            self.port = port
        else:
            resources = self.resourceManager.list_resources()
            matching_resources = []
            for device in resources:
                if 'DS1Z' in device:
                    matching_resources.append(device)
            if matching_resources:
                self.port = matching_resources[0]
            else:
                raise IOError('Matching device not found')
        try:
            self.instrument = self.resourceManager.open_resource(self.port)
            self.instrument.timeout = 10000
            self.instrument.chunk_size = 1048576
        except pyvisa.errors.VisaIOError:
            raise IOError('Could not connect to DS1104Z at {}'.format(self.port))

    def run(self):
        self.write(':RUN')

    def stop(self):
        self.write(':STOP')

    def autoscale(self):
        self.write(':AUToscale')

    def clear(self):
        self.write(':CLEar')

    def single(self):
        self.write(':SINGle')

    def trigger_force(self):
        self.write(':TFORce')

    def acquire_averages(self, count=None):
        if count is not None:
            self.write(':ACQuire:AVERages {}'.format(count))
        else:
            return self.query(':ACQuire:AVERages?')

    def acquire_memory_depth(self, memory_depth=None):
        if memory_depth is not None:
            if memory_depth == 'AUTO':
                self.write(':ACQuire:MDEPth AUTO')
            else:
                self.write(':ACQuire:MDEPth {}'.format(memory_depth))
        else:
            return int(self.query('ACQuire:MDEPth?'))

    def acquire_type(self, acquisition_type=None):
        if acquisition_type is not None:
            self.write(':ACQuire:TYPE {}'.format(acquisition_type))
        else:
            return self.query(':ACQuire:TYPE?')

    def acquire_sample_rate(self):
        return float(self.query(':ACQuire:SRATe?'))

    def channel_bandwidth_limit(self, channel, limit_type=None):
        if limit_type is not None:
            self.write(':CHANnel{}:BWLimit {}'.format(channel, limit_type))
        else:
            return self.query('CHANnel{}.BWLimit?'.format(channel))

    def channel_coupling(self, channel, coupling=None):
        if coupling is not None:
            self.write(':CHANnel{}:COUPling {}'.format(channel, coupling))
        else:
            return self.query(':CHANnel{}:COUPling?'.format(channel))

    def channel_display(self, channel, display=None):
        if display is not None:
            self.write(':CHANnel{}:DISPlay {}'.format(channel, display))
        else:
            result = self.query(':CHANnel{}:DISPlay?'.format(channel))
            if int(result):
                return True
            else:
                return False

    def channel_invert(self, channel, invert=None):
        if invert is not None:
            self.write(':CHANnel{}:INVert {}'.format(channel, invert))
        else:
            result = self.query(':CHANnel{}:INVert?'.format(channel))
            if int(result):
                return True
            else:
                return False

    def channel_offset(self, channel, offset=None):
        if offset is not None:
            self.write(':CHANnel{}:OFFSet {}'.format(channel, offset))
        else:
            return float(self.query(':CHANnel{}:OFFSet?'.format(channel)))

    def channel_range(self, channel, set_range=None):
        if set_range is not None:
            self.write(':CHANnel{}:RANGe {}'.format(channel, set_range))
        else:
            return float(self.query(':CHANnel{}:RANGe?'.format(channel)))

    def channel_scale(self, channel, scale=None):
        if scale is not None:
            self.write(':CHANnel{}:SCALe {}'.format(channel, scale))
        else:
            return float(self.query(':CHANnel{}:SCALe?'.format(channel)))

    def channel_probe(self, channel, probe=None):
        if probe is not None:
            self.write(':CHANnel{}:PROBe {}'.format(channel, probe))
        else:
            return float(self.query(':CHANnel{}:PROBe?'.format(channel)))

    def channel_vernier(self, channel, vernier=None):
        if vernier is not None:
            self.write(':CHANnel{}:VERNier {}'.format(channel, vernier))
        else:
            result = self.query(':CHANnel{}:VERNier?'.format(channel))
            if int(result):
                return True
            else:
                return False

    def timebase_scale(self, scale=None):
        if scale is not None:
            self.write(':TIMebase:SCALe {}'.format(scale))
        else:
            return float(self.query(':TIMebase:SCALe?'))

    def trigger_status(self):
        return self.query(':TRIGger:STATus?')

    def trigger_sweep(self, sweep=None):
        if sweep is not None:
            self.write(':TRIGger:SWEep {}'.format(sweep))
        else:
            return self.query(':TRIGger:SWEep?')

    def trigger_edge_source(self, source=None):
        if source is not None:
            self.write(':TRIGger:EDGe:SOURce {}'.format(source))
        else:
            return self.query(':TRIGger:EDGe:SOURce?')

    def trigger_edge_slope(self, slope=None):
        if slope is not None:
            self.write(':TRIGger:EDGe:SOURce {}'.format(slope))
        else:
            return self.query(':TRIGger:EDGe:SLOPe?')

    def trigger_edge_level(self, level=None):
        if level is not None:
            self.write(':TRIGger:EDGe:LEVel {}'.format(level))
        else:
            return float(self.query(':TRIGger:EDGe:LEVel?'))

    def waveform_source(self, source=None):
        if source is not None:
            self.write(':WAVeform:SOURce {}'.format(source))
        else:
            return self.query(':WAVeform:SOURce?')

    def waveform_mode(self, mode=None):
        if mode is not None:
            self.write(':WAVeform:MODE {}'.format(mode))
        else:
            return self.query(':WAVeform:MODE?')

    def waveform_format(self, set_format=None):
        if set_format is not None:
            self.write(':WAVeform:FORMat {}'.format(set_format))
        else:
            return self.query(':WAVeform:FORMat?')

    def waveform_data(self, channel):
        self.stop()
        # set data transfer mode
        self.waveform_source('CHAN{}'.format(channel))
        self.waveform_mode('RAW')
        self.waveform_format('BYTE')
        # get scaling and sample information
        y_origin = int(self.query(':WAVeform:YORigin?'))
        y_reference = int(self.query(':WAVeform:YREFerence?'))
        y_increment = float(self.query(':WAVeform:YINCrement?'))
        memory_depth = self.acquire_memory_depth()
        sample_rate = self.acquire_sample_rate()
        # initalize output
        output_data = list()
        # initialize memory position
        memory_position_start = 1
        if memory_depth >= 250000:
            memory_position_stop = 250000
        else:
            memory_position_stop = memory_depth
        # loop through sample memory in chunks
        while memory_position_stop <= memory_depth:
            self.write(':WAVeform:STARt {}'.format(memory_position_start))
            self.write(':WAVeform:STOP {}'.format(memory_position_stop))
            try:
                waveform_data_chunk = self.instrument.query_binary_values(
                    ':WAVeform:DATA?', datatype='B', data_points=(memory_position_stop - memory_position_start))
                output_data.append(waveform_data_chunk)
            except pyvisa.errors.VisaIOError:
                raise OSError('Communication error')
            # increment memory position
            memory_position_start = memory_position_stop + 1
            memory_position_stop = memory_position_start + 249999
            if memory_position_stop > memory_depth:
                memory_position_stop = memory_depth
            if memory_position_start > memory_depth:
                break
        # flatten and scale sample data
        output_data = [item for sublist in output_data for item in sublist]
        output_data = list(map(float, output_data))
        for sample in range(len(output_data)):
            output_data[sample] = float((output_data[sample] - y_origin - y_reference) * y_increment)
        # generate sample times
        sample_times = numpy.linspace(0.0, ((1 / sample_rate) * memory_depth), memory_depth)

        return output_data, sample_times

    def phase_offset(self, channel1, channel2, frequency):
        channel1_data = self.waveform_data(channel1)[0]
        channel2_data = self.waveform_data(channel2)[0]
        sample_rate = self.acquire_sample_rate()
        memory_depth = self.acquire_memory_depth()

        correlation = signal.correlate(channel1_data, channel2_data)
        dt = numpy.arange(1 - memory_depth, memory_depth)
        shift = dt[correlation.argmax()]
        phase = (360 * (shift * (1 / sample_rate))) / (1 / frequency)

        return phase
