package scd4x // import "code.nkcmr.net/adadriver/scd4x"

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"io/ioutil"
	"log"
	"math"
	"sync"
	"time"

	"periph.io/x/conn/v3/i2c"
)

// SCD41I2CAddr is the default address for the SCD-41 sensor
const SCD41I2CAddr = 0x62

var localLog = log.New(ioutil.Discard, "[scd4x_driver] ", log.Lmicroseconds)

// SetLogOutput sets the output for the debug log to the provided writer.
// os.Stderr is a good choice if there is confusion about what to use.
//
// ioutil.Discard can be passed to disable logging again.
func SetLogOutput(w io.Writer) {
	localLog.SetOutput(w)
}

const (
	_SCD4X_REINIT                           = 0x3646
	_SCD4X_FACTORYRESET                     = 0x3632
	_SCD4X_FORCEDRECAL                      = 0x362F
	_SCD4X_SELFTEST                         = 0x3639
	_SCD4X_DATAREADY                        = 0xE4B8
	_SCD4X_STOPPERIODICMEASUREMENT          = 0x3F86
	_SCD4X_STARTPERIODICMEASUREMENT         = 0x21B1
	_SCD4X_STARTLOWPOWERPERIODICMEASUREMENT = 0x21AC
	_SCD4X_READMEASUREMENT                  = 0xEC05
	_SCD4X_SERIALNUMBER                     = 0x3682
	_SCD4X_GETTEMPOFFSET                    = 0x2318
	_SCD4X_SETTEMPOFFSET                    = 0x241D
	_SCD4X_GETALTITUDE                      = 0x2322
	_SCD4X_SETALTITUDE                      = 0x2427
	_SCD4X_SETPRESSURE                      = 0xE000
	_SCD4X_PERSISTSETTINGS                  = 0x3615
	_SCD4X_GETASCE                          = 0x2313
	_SCD4X_SETASCE                          = 0x2416
	_SCD4X_MEASURE_SINGLE_SHOT              = 0x219D
	_SCD4X_MEASURE_SINGLE_SHOT_RHT_ONLY     = 0x2196
)

// SCD4x is the instance of a particular sensor.
type SCD4x struct {
	b *i2c.Dev
}

var mutex sync.Mutex

// Open will initialize a SCD-4x sensor
func Open(i2cAddr uint16, bus i2c.Bus) *SCD4x {
	mutex.Lock()
	defer mutex.Unlock()

	dev := &i2c.Dev{Addr: i2cAddr, Bus: bus}

	return &SCD4x{b: dev}
}

// StartPeriodicMeasurement starts periodic measurement, signal update interval
// is 5 seconds
func (s *SCD4x) StartPeriodicMeasurement() error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: StartPeriodicMeasurement", s)

	if err := s.sendCommand(_SCD4X_STARTPERIODICMEASUREMENT, -1); err != nil {
		return fmt.Errorf("_SCD4X_STARTPERIODICMEASUREMENT failed: %w", err)
	}
	return nil
}

// Measurement contains the data that is yielded by the sensor.
type Measurement struct {
	CO2  uint16  // ppm
	Temp float32 // celcius
	RH   float32 // humidity, percentage
}

// ReadMeasurement will read sensor output. The measurement data can only be
// read out once per signal update interval as the buffer is emptied upon
// read-out. If no data is available in the buffer, the sensor returns a NACK.
// To avoid a NACK response, the get_data_ready_status can be issued to check
// data status (see GetDataReadyStatus for further details).
func (s *SCD4x) ReadMeasurement() (Measurement, error) {
	mutex.Lock()
	defer mutex.Unlock()

	localLog.Printf("%T: ReadMeasurement", s)

	if err := s.sendCommand(_SCD4X_READMEASUREMENT, time.Millisecond); err != nil {
		return Measurement{}, fmt.Errorf("_SCD4X_READMEASUREMENT failed: %w", err)
	}

	out := Measurement{}

	_buf, err := s.readData(9)
	if err != nil {
		return Measurement{}, fmt.Errorf("failed to read raw data: %w", err)
	}
	buf := bytes.NewBuffer(_buf)

	co2b, err := readValid16(buf)
	if err != nil {
		return Measurement{}, fmt.Errorf("failed to read co2 from reply: %w", err)
	}
	out.CO2 = binary.BigEndian.Uint16(co2b)

	tempRaw, err := readValid16(buf)
	if err != nil {
		return Measurement{}, fmt.Errorf("failed to read temp from reply: %w", err)
	}
	out.Temp = float32(-45) + float32(175)*float32(binary.BigEndian.Uint16(tempRaw))/float32(math.Pow(2, 16))

	humRaw, err := readValid16(buf)
	if err != nil {
		return Measurement{}, fmt.Errorf("failed to read humidity from reply: %w", err)
	}
	out.RH = float32(100) * float32(binary.BigEndian.Uint16(humRaw)) / float32(math.Pow(2, 16))

	return out, nil
}

// StopPeriodicMeasurement stops periodic measurement to change the sensor
// configuration or to save power. Note that the sensor will only respond to
// other commands after waiting 500 ms after issuing the
// stop_periodic_measurement command.
func (s *SCD4x) StopPeriodicMeasurement() error {
	mutex.Lock()
	defer mutex.Unlock()

	localLog.Printf("%T: stopPeriodicMeasurement", s)
	if err := s.sendCommand(_SCD4X_STOPPERIODICMEASUREMENT, time.Millisecond*500); err != nil {
		return fmt.Errorf("_SCD4X_STOPPERIODICMEASUREMENT failed: %w", err)
	}

	return nil
}

// SetTemperatureOffset: The temperature offset has no influence on the SCD4x
// CO2 accuracy. Setting the temperature offset of the SCD4x inside the customer
// device correctly allows the user to leverage the RH and T output signal. Note
// that the temperature offset can depend on various factors such as the SCD4x
// measurement mode, self-heating of close components, the ambient temperature
// and air flow. Thus, the SCD4x temperature offset should be determined inside
// the customer device under its typical operation conditions (including the
// operation mode to be used in the application) and in thermal equilibrium. Per
// default, the temperature offset is set to 4° C. To save the setting to the
// EEPROM, the persist setting (see SCD4x.PersistSetting) command must be
// issued. Equation (1) shows how the characteristic temperature offset can be
// obtained.
//
// Equation (1):
//
//     T_offset = T_scd40 - T_reference + T_offset_previous
//
func (s *SCD4x) SetTemperatureOffset(offset uint16) error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: SetTemperatureOffset", s)

	if err := s.sendCommandArg(_SCD4X_SETTEMPOFFSET, offset, time.Millisecond); err != nil {
		return fmt.Errorf("_SCD4X_SETTEMPERATUREOFFSET failed: %w", err)
	}

	return nil
}

// GetTemperatureOffset returns the current temperature offset.
func (s *SCD4x) GetTemperatureOffset() (uint16, error) {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: GetTemperatureOffset", s)

	if err := s.sendCommand(_SCD4X_GETTEMPOFFSET, time.Millisecond); err != nil {
		return 0, fmt.Errorf("_SCD4X_GETTEMPOFFSET failed: %w", err)
	}

	_buf, err := s.readData(3)
	if err != nil {
		return 0, fmt.Errorf("failed to read raw data: %w", err)
	}
	buf := bytes.NewBuffer(_buf)

	offset, err := readValid16(buf)
	if err != nil {
		return 0, fmt.Errorf("failed to read offset from reply: %w", err)
	}

	return binary.BigEndian.Uint16(offset), nil
}

// SetSensorAltitude: Reading and writing of the sensor altitude must be done
// while the SCD4x is in idle mode. Typically, the sensor altitude is set once
// after device installation. To save the setting to the EEPROM, the persist
// setting (see PersistSettings) command must be issued. Per default, the sensor
// altitude is set to 0 meter above sea-level.
func (s *SCD4x) SetSensorAltitude(masl uint16) error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: SetSensorAltitude", s)

	if err := s.sendCommandArg(_SCD4X_SETALTITUDE, masl, time.Millisecond); err != nil {
		return fmt.Errorf("_SCD4X_SETALTITUDE failed: %w", err)
	}

	return nil
}

// GetSensorAltitude returns the current sensor altitude setting.
func (s *SCD4x) GetSensorAltitude() (uint16, error) {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: GetSensorAltitude", s)

	if err := s.sendCommand(_SCD4X_GETALTITUDE, time.Millisecond); err != nil {
		return 0, fmt.Errorf("_SCD4X_GETALTITUDE failed: %w", err)
	}

	_buf, err := s.readData(3)
	if err != nil {
		return 0, fmt.Errorf("failed to read raw data: %w", err)
	}
	buf := bytes.NewBuffer(_buf)

	masl, err := readValid16(buf)
	if err != nil {
		return 0, fmt.Errorf("failed to read masl from reply: %w", err)
	}

	return binary.BigEndian.Uint16(masl), nil
}

// SetAmbientPressure: The set_ambient_pressure command can be sent during
// periodic measurements to enable continuous pressure compensation. Note that
// setting an ambient pressure using set_ambient_pressure overrides any pressure
// compensation based on a previously set sensor altitude.
//
// The input unit is in hPa (hectopascals). So, passing 987 will set the ambient
// pressure to 98,700 Pa (or ~99 kPa)
func (s *SCD4x) SetAmbientPressure(hpa uint16) error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: SetAmbientPressure(%d)", s, hpa)

	if err := s.sendCommandArg(_SCD4X_SETPRESSURE, hpa, time.Millisecond); err != nil {
		return fmt.Errorf("_SCD4X_SETAMBIENTPRESSURE failed: %w", err)
	}

	return nil
}

// PerformForcedRecalibration: To successfully conduct an accurate forced
// recalibration, the following steps need to be carried out:
// 1. Operate the SCD4x in the operation mode later used in normal sensor
// operation (periodic measurement, low power periodic measurement or single
// shot) for > 3 minutes in an environment with homogenous and constant CO2
// concentration.
// 2. Issue stop_periodic_measurement. Wait 500 ms for the stop command to
// complete.
// 3. Subsequently issue the perform_forced_recalibration command and optionally
// read out the FRC correction (i.e. the magnitude of the correction) after
// waiting for 400 ms for the command to complete.
//
// A return value of 0xffff indicates that the forced recalibration has failed.
// Note that the sensor will fail to perform a forced recalibration if it was
// not operated before sending the command. Please make sure that the sensor is
// operated at the voltage desired for the application when applying the forced
// recalibration sequence.
func (s *SCD4x) PerformForcedRecalibration(targetCO2Concetration uint16) (frcCorrection uint16, _ error) {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: PerformForcedRecalibration(0x%x)", s, targetCO2Concetration)

	if err := s.sendCommandArg(_SCD4X_FORCEDRECAL, targetCO2Concetration, time.Millisecond*400); err != nil {
		return 0, fmt.Errorf("_SCD4X_FORCEDRECAL failed: %w", err)
	}
	_buf, err := s.readData(3)
	if err != nil {
		return 0, fmt.Errorf("failed to read raw data: %w", err)
	}
	buf := bytes.NewBuffer(_buf)
	_result, err := readValid16(buf)
	if err != nil {
		return 0, fmt.Errorf("failed to validate sensor response: %w", err)
	}
	result := binary.BigEndian.Uint16(_result)
	if result == 0xFFFF {
		return 0, fmt.Errorf("sensor reported failure for forced recalibration")
	}
	return result - 0x8000, nil
}

// SetAutomaticSelfCalibration: Set the current state (enabled / disabled) of
// the automatic self-calibration. By default, ASC is enabled. To save the
// setting to the EEPROM, the persist_setting (see PersistSettings) command must
// be issued.
func (s *SCD4x) SetAutomaticSelfCalibrationEnabled(enabled bool) error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: SetAutomaticSelfCalibration(%t)", s, enabled)

	arg := uint16(0)
	if enabled {
		arg = 1
	}

	if err := s.sendCommandArg(_SCD4X_SETASCE, arg, time.Millisecond); err != nil {
		return fmt.Errorf("_SCD4X_SETASCE failed: %w", err)
	}

	return nil
}

// GetAutomaticSelfCalibrationEnabled returns the current state of ASC
func (s *SCD4x) GetAutomaticSelfCalibrationEnabled() (bool, error) {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: GetAutomaticSelfCalibrationEnabled", s)

	if err := s.sendCommand(_SCD4X_GETASCE, time.Millisecond); err != nil {
		return false, fmt.Errorf("_SCD4X_GETASCE failed: %w", err)
	}

	_buf, err := s.readData(3)
	if err != nil {
		return false, fmt.Errorf("failed to read raw data: %w", err)
	}
	buf := bytes.NewBuffer(_buf)

	_result, err := readValid16(buf)
	if err != nil {
		return false, fmt.Errorf("failed to validate sensor response: %w", err)
	}
	result := binary.BigEndian.Uint16(_result)
	return result == 1, nil
}

// StartLowPowerPeriodicMeasurement will start low power periodic measurement,
// signal update interval is approximately 30 seconds.
func (s *SCD4x) StartLowPowerPeriodicMeasurement() error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: StartLowPowerPeriodicMeasurement", s)

	if err := s.sendCommand(_SCD4X_STARTLOWPOWERPERIODICMEASUREMENT, -1); err != nil {
		return fmt.Errorf("_SCD4X_STARTLOWPOWERPERIODICMEASUREMENT failed: %w", err)
	}
	return nil
}

// GetDataReadyStatus will indicate if the sensor has a measurement that is
// ready to be read.
func (s *SCD4x) GetDataReadyStatus() (bool, error) {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: getDataReadyStatus", s)

	if err := s.sendCommand(_SCD4X_DATAREADY, time.Millisecond); err != nil {
		return false, fmt.Errorf("failed to send send data ready command: %w", err)
	}
	_buf, err := s.readData(3)
	if err != nil {
		return false, fmt.Errorf("failed to read raw data: %w", err)
	}
	localLog.Printf("%T: getDataReadStatus: sensory reply: %#v", s, _buf)
	buf := bytes.NewBuffer(_buf)
	_ready, err := readValid16(buf)
	if err != nil {
		return false, fmt.Errorf("failed to ")
	}
	return !((_ready[0]&0x07 == 0) && (_ready[1] == 0)), nil
}

// PersistSettings: Configuration settings such as the temperature offset,
// sensor altitude and the ASC enabled/disabled parameter are by default stored
// in the volatile memory (RAM) only and will be lost after a power-cycle.
// The persist_settings command stores the current configuration in the EEPROM
// of the SCD4x, making them persistent across power-cycling. To avoid
// unnecessary wear of the EEPROM, the persist_settings command should only be
// sent when persistence is required and if actual changes to the configuration
// have been made. The EEPROM is guaranteed to endure at least 2000 write cycles
// before failure. Note that field calibration history (i.e. FRC and ASC, see
// chapter 3.7) is automatically stored in a separate EEPROM dimensioned for the
// specified sensor lifetime.
func (s *SCD4x) PersistSettings() error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: PersistSettings", s)

	if err := s.sendCommand(_SCD4X_PERSISTSETTINGS, time.Millisecond*800); err != nil {
		return fmt.Errorf("failed to send persist settings command: %w", err)
	}
	return nil
}

// GetSerialNumber: Reading out the serial number can be used to identify the
// chip and to verify the presence of the sensor. The get serial number command
// returns 3 words, and every word is followed by an 8-bit CRC checksum.
// Together, the 3 words constitute a unique serial number with a length of 48
// bits
func (s *SCD4x) GetSerialNumber() (uint64, error) {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: GetSerialNumber", s)

	if err := s.sendCommand(_SCD4X_SERIALNUMBER, time.Millisecond); err != nil {
		return 0, fmt.Errorf("failed to send get serial number command: %w", err)
	}
	_buf, err := s.readData(9)
	if err != nil {
		return 0, fmt.Errorf("failed to read raw data: %w", err)
	}
	buf := bytes.NewBuffer(_buf)
	words, err := readMultipleValid16(buf, 3)
	if err != nil {
		return 0, fmt.Errorf("failed to validate sensor response: %w", err)
	}
	if len(words) != 3 {
		return 0, fmt.Errorf("internal error: unexpected number of words in sensor response data")
	}
	return uint64(words[0])<<32 | uint64(words[1])<<16 | uint64(words[2]), nil
}

// PerformSelfTest can be used as an end-of-line test to check sensor
// functionality and the customer power supply to the sensor.
func (s *SCD4x) PerformSelfTest() error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: PerformSelfTest", s)

	if err := s.sendCommand(_SCD4X_SELFTEST, time.Millisecond*10_000); err != nil {
		return fmt.Errorf("_SCD4X_SELFTEST failed: %w", err)
	}
	_buf, err := s.readData(3)
	if err != nil {
		return fmt.Errorf("failed to read data raw data: %w", err)
	}
	buf := bytes.NewBuffer(_buf)
	_result, err := readValid16(buf)
	if err != nil {
		return fmt.Errorf("failed to validate sensor response: %w", err)
	}
	if binary.BigEndian.Uint16(_result) != 0 {
		return fmt.Errorf("sensor malfunction detected")
	}
	return nil
}

// PerformFactoryReset command resets all configuration settings stored in
// the EEPROM and erases the FRC and ASC algorithm history.
func (s *SCD4x) PerformFactoryReset() error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: performFactoryReset", s)

	if err := s.sendCommand(_SCD4X_FACTORYRESET, time.Millisecond*1200); err != nil {
		return fmt.Errorf("_SCD4X_FACTORYRESET failed: %w", err)
	}

	return nil
}

// Reinit reinitializes the sensor by reloading user settings from EEPROM.
// Before sending the Reinit command, the stop measurement command must be
// issued. If the Reinit command does not trigger the desired re-initialization,
// a power-cycle should be applied to the SCD4x.
func (s *SCD4x) Reinit() error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: Reinit", s)

	if err := s.sendCommand(_SCD4X_REINIT, time.Millisecond*20); err != nil {
		return fmt.Errorf("_SCD4X_REINIT failed: %w", err)
	}

	return nil
}

// MeasureSingleShot triggers on-demand measurement of CO2 concentration,
// relative humidity and temperature. The sensor output is read using the
// ReadMeasurement command
func (s *SCD4x) MeasureSingleShot() error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: MeasureSingleShot", s)

	if err := s.sendCommand(_SCD4X_MEASURE_SINGLE_SHOT, time.Millisecond*5000); err != nil {
		return fmt.Errorf("_SCD41_MEASURE_SINGLE_SHOT failed: %w", err)
	}

	return nil
}

// On-demand measurement of relative humidity and temperature only. The sensor
// output is read using the ReadMeasurement command. CO2 output is returned as
// 0 ppm.
func (s *SCD4x) MeasureSingleShotRHTOnly() error {
	mutex.Lock()
	defer mutex.Unlock()
	localLog.Printf("%T: MeasureSingleShotRHTOnly", s)

	if err := s.sendCommand(_SCD4X_MEASURE_SINGLE_SHOT_RHT_ONLY, time.Millisecond*50); err != nil {
		return fmt.Errorf("_SCD41_MEASURE_SINGLE_SHOT_RHT_ONLY failed: %w", err)
	}

	return nil
}

func (s *SCD4x) readData(len int) ([]byte, error) {
	data := make([]byte, len)
	if err := s.b.Tx(nil, data); err != nil {
		return nil, fmt.Errorf("Tx failed: %w", err)
	}
	return data, nil
}

func (s *SCD4x) sendCommand(cmd uint16, sleep time.Duration) error {
	localLog.Printf("%T: sendCommand: 0x%x", s, cmd)
	cmdData := make([]byte, 2)
	binary.BigEndian.PutUint16(cmdData, cmd)
	if err := s.writeAndWait(cmdData, sleep); err != nil {
		return err
	}
	return nil
}

func (s *SCD4x) sendCommandArg(cmd uint16, arg uint16, sleep time.Duration) error {
	localLog.Printf("%T: sendCommandArg: cmd=0x%x arg=0x%x", s, cmd, arg)
	cmdData := make([]byte, 2)
	argData := make([]byte, 2)
	binary.BigEndian.PutUint16(cmdData, cmd)
	binary.BigEndian.PutUint16(argData, arg)
	write := []byte{cmdData[0], cmdData[1], argData[0], argData[1], crc(argData)}
	return s.writeAndWait(write, sleep)
}

func (s *SCD4x) writeAndWait(data []byte, sleep time.Duration) error {
	localLog.Printf("%T: writeAndWait: %#v", s, data)
	if err := s.b.Tx(data, nil); err != nil {
		return err
	}
	if sleep > time.Second {
		localLog.Printf("%T: writeAndWait: waiting %s for response", s, sleep)
	}
	time.Sleep(sleep)
	return nil
}

func readValid16(buf *bytes.Buffer) ([]byte, error) {
	data := buf.Next(2)
	crc8, err := buf.ReadByte()
	if err != nil {
		return nil, err
	}
	if crc(data) != crc8 {
		return nil, fmt.Errorf("crc error, expected %x got %x", crc(data), crc8)
	}
	return data, nil
}

func readMultipleValid16(buf *bytes.Buffer, n int) ([]uint16, error) {
	result := make([]uint16, n)
	for i := 0; i < n; i++ {
		w, err := readValid16(buf)
		if err != nil {
			return nil, fmt.Errorf("failed to verify word %d in response: %w", i, err)
		}
		result[i] = binary.BigEndian.Uint16(w)
	}
	return result, nil
}

// #define CRC8_POLYNOMIAL 0x31
// #define CRC8_INIT 0xFF
// uint8_t sensirion_common_generate_crc(const uint8_t* data, uint16_t count) {
//     uint16_t current_byte;
//     uint8_t crc = CRC8_INIT;
//     uint8_t crc_bit;
//     /* calculates 8-Bit checksum with given polynomial */
//     for (current_byte = 0; current_byte < count; ++current_byte) {
//         crc ^= (data[current_byte]);
//         for (crc_bit = 8; crc_bit > 0; --crc_bit) {
//             if (crc & 0x80)
//                 crc = (crc << 1) ^ CRC8_POLYNOMIAL;
//             else
//                 crc = (crc << 1);
//         }
//     }
//     return crc;
// }
func crc(buffer []byte) byte {
	const crc8Polynomial = 0x31
	const crc8Init = 0xFF

	crc := byte(crc8Init)
	for _, b := range buffer {
		crc ^= b
		for i := 0; i < 8; i++ {
			if (crc & 0x80) != 0 {
				crc = (crc << 1) ^ crc8Polynomial
			} else {
				crc = crc << 1
			}
		}
	}
	return crc & 0xFF
}
