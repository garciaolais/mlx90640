// Ported from https://github.com/adafruit/Adafruit_CircuitPython_MLX90640/blob/main/adafruit_mlx90640.py
package mlx90640

import (
	"encoding/binary"
	"log"
	"math"

	"periph.io/x/periph/conn/i2c"
	"periph.io/x/periph/conn/i2c/i2creg"
	"periph.io/x/periph/host"
)

const (
	Sampling_0_5_Hz = 0x00
	Sampling_01_Hz  = 0x01
	Sampling_02_Hz  = 0x02
	Sampling_04_Hz  = 0x03
	Sampling_08_Hz  = 0x04
	Sampling_16_Hz  = 0x05
	Sampling_32_Hz  = 0x06
	Sampling_64_Hz  = 0x07
)

func GetHellow() string {
	return "hellow"
}

const (
	WIDTH             = 32
	HEIGHT            = 24
	PIXELCOUNT        = WIDTH * HEIGHT
	OPENAIR_TA_SHIFT  = 8
	SCALEALPHA        = 0.000001
	DefaultI2cAddress = 0x33
)

type Mlx90640 struct {
	i2c                *i2c.Dev
	busCloser          i2c.BusCloser
	width              int
	height             int
	pixelCount         int
	OPENAIR_TA_SHIFT   int
	SCALEALPHA         float64
	_frameData         [834]uint16
	_eeData            []uint16
	_alpha             []int
	_alphaPTAT         float64
	_alphaScale        byte
	_brokenPixels      [5]uint16
	_calibrationModeEE int
	_cpAlpha           [2]float64
	_cpKta             float64
	_cpKv              float64
	_cpOffset          [2]int
	_ct                [5]int
	_gainEE            int16
	_ilChessC          [3]float64
	_ksTa              float64
	_ksTo              [5]float64
	_kta               []byte
	_ktaScale          byte
	_ktPTAT            float64
	_kVdd              int16
	_kv                []int8
	_kvPTAT            float64
	_kvScale           byte
	_offset            []int
	_outlierPixels     [5]uint16
	_resolutionEE      byte
	_tgc               float64
	_vdd25             int16
	_vPTAT25           int16
}

func New(addr string) *Mlx90640 {
	mlx90640 := &Mlx90640{
		width:            WIDTH,
		height:           HEIGHT,
		pixelCount:       PIXELCOUNT,
		OPENAIR_TA_SHIFT: OPENAIR_TA_SHIFT,
		SCALEALPHA:       SCALEALPHA,
		_eeData:          make([]uint16, 832),
		_alpha:           make([]int, PIXELCOUNT),
		_kta:             make([]byte, PIXELCOUNT),
		_kv:              make([]int8, PIXELCOUNT),
		_offset:          make([]int, PIXELCOUNT),
	}

	if _, err := host.Init(); err != nil {
		log.Fatal(err)
	}

	var err error
	mlx90640.busCloser, err = i2creg.Open(addr)
	if err != nil {
		log.Fatal(err)
	}

	mlx90640.i2c = &i2c.Dev{Addr: 0x33, Bus: mlx90640.busCloser}
	mlx90640.DumpEE()
	mlx90640.ExtractVDDParameters()
	mlx90640.ExtractPTATParameters()
	mlx90640.ExtractGainParameters()
	mlx90640.ExtractTgcParameters()
	mlx90640.ExtractResolutionParameters()
	mlx90640.ExtractKsTaParameters()
	mlx90640.ExtractKsToParameters()
	mlx90640.ExtractCPParameters()
	mlx90640.ExtractAlphaParameters()
	mlx90640.ExtractOffsetParameters()
	mlx90640.ExtractKtaPixelParameters()
	mlx90640.ExtractKvPixelParameters()
	mlx90640.ExtractCILCParameters()
	mlx90640.ExtractDeviatingPixels()

	return mlx90640
}

func (m *Mlx90640) Close() {
	m.busCloser.Close()
}

func (m *Mlx90640) SetSampling(mode int) {
	controlRegister := make([]uint16, 1)

	value := (mode & 0x7) << 7

	controlRegister = m.readRegisters16(0x800D, 1)
	value = int(controlRegister[0]&0xFC7F) | value
	m.write16(0x800D, value)
}

func (m *Mlx90640) GetFrame() []float64 {
	framebuf := make([]float64, m.pixelCount)
	emissivity := 0.95

	for i := 0; i < 2; i++ {
		if m.GetFrameData() < 0 {
			panic("Frame data error")
		}

		tr := m.GetTa() - OPENAIR_TA_SHIFT
		m.CalculateTo(float64(emissivity), tr, framebuf)
	}

	return framebuf
}

func (m *Mlx90640) CalculateTo(emissivity float64, tr float64, result []float64) []float64 {
	subPage := uint16(m._frameData[833])
	vdd := m.GetVdd()
	ta := m.GetTa()

	ta4 := ta + 273.15
	ta4 = ta4 * ta4
	ta4 = ta4 * ta4
	tr4 := tr + 273.15
	tr4 = tr4 * tr4
	tr4 = tr4 * tr4
	taTr := (tr4 - (tr4 - ta4)) / emissivity

	ktaScale := math.Pow(2, float64(m._ktaScale))
	kvScale := math.Pow(2, float64(m._kvScale))
	alphaScale := math.Pow(2, float64(m._alphaScale))

	alphaCorrR := [4]float64{}
	alphaCorrR[0] = 1 / (1 + m._ksTo[0]*40)
	alphaCorrR[1] = 1
	alphaCorrR[2] = (1 + m._ksTo[1]*float64(m._ct[2]))
	alphaCorrR[3] = alphaCorrR[2] * (1 + m._ksTo[2]*float64(m._ct[3]-m._ct[2]))

	//------------------------- Gain calculation -----------------------------------
	gain := float64(m._frameData[778])
	gain = float64(m._gainEE) / gain

	//------------------------- To calculation -------------------------------------
	mode := byte((m._frameData[832] & 0x1000) >> 5)

	irDataCP := [2]float64{}
	irDataCP[0] = float64(m._frameData[776])
	irDataCP[1] = float64(m._frameData[808])
	for i := 0; i < 2; i++ {
		if irDataCP[i] > 32767 {
			irDataCP[i] = irDataCP[i] - 65536
		}

		irDataCP[i] = gain * irDataCP[i]
	}

	irDataCP[0] -= float64(m._cpOffset[0]) * (1 + m._cpKta*(ta-25.0)) * (1 + m._cpKv*(vdd-3.3))
	if int(mode) == m._calibrationModeEE {
		irDataCP[1] -= float64(m._cpOffset[1]) * (1 + m._cpKta*(ta-25)) * (1 + m._cpKv*(vdd-3.3))
	} else {
		irDataCP[1] -= (float64(m._cpOffset[1]) + m._ilChessC[0]) * (1 + m._cpKta*(ta-25)) * (1 + m._cpKv*(vdd-3.3))
	}

	for pixelNumber := 0; pixelNumber < 768; pixelNumber++ {
		ilPattern := int8(pixelNumber/32 - (pixelNumber/64)*2)
		chessPattern := byte(ilPattern ^ int8(pixelNumber-(pixelNumber/2)*2))
		conversionPattern := float64(((pixelNumber+2)/4 - (pixelNumber+3)/4 + (pixelNumber+1)/4 - pixelNumber/4) * (1 - 2*int(ilPattern)))

		var pattern byte = 0
		if mode == 0 {
			pattern = byte(ilPattern)
		} else {
			pattern = chessPattern
		}

		if uint16(pattern) == m._frameData[833] {
			irData := float64(m._frameData[pixelNumber])
			if irData > 32767 {
				irData -= 65536
			}
			irData *= gain

			kta := float64(m._kta[pixelNumber]) / ktaScale
			kv := float64(m._kv[pixelNumber]) / kvScale
			irData = irData - float64(float64(m._offset[pixelNumber])*(1+kta*(ta-25)))*(1+kv*(vdd-3.3))

			if int(mode) != m._calibrationModeEE {
				irData = irData + m._ilChessC[2]*(2*float64(ilPattern)-1) - m._ilChessC[1]*conversionPattern
			}

			irData = irData - m._tgc*irDataCP[subPage]
			irData = irData / emissivity

			alphaCompensated := float64(SCALEALPHA * alphaScale / float64(m._alpha[pixelNumber]))
			alphaCompensated = float64(alphaCompensated * (1 + m._ksTa*(ta-25)))

			sx := alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated*taTr)
			sx = math.Sqrt(math.Sqrt(sx)) * float64(m._ksTo[1])

			to := math.Sqrt(math.Sqrt(float64(irData/(alphaCompensated*(1-m._ksTo[1]*273.15)+sx)+taTr))) - 273.15

			var _range byte = 0
			if int(to) < m._ct[1] {
				_range = 0
			} else if int(to) < m._ct[2] {
				_range = 1
			} else if int(to) < m._ct[3] {
				_range = 2
			} else {
				_range = 3
			}

			to = math.Sqrt(math.Sqrt(float64(irData/(alphaCompensated*alphaCorrR[_range]*(1+m._ksTo[_range]*(to-float64(m._ct[_range]))))+taTr))) - 273.15

			result[pixelNumber] = to
		}
	}

	return result
}

func (m *Mlx90640) GetFrameData() uint16 {
	statusRegister := make([]uint16, 0)
	dataReady := 0
	for dataReady == 0 {
		statusRegister = m.readRegisters16(0x8000, 1)
		dataReady = int(statusRegister[0] & 0x0008)
	}

	cnt := 0
	framedata := make([]uint16, 832)

	for (dataReady != 0) && (cnt < 5) {
		m.write16(0x8000, 0x0030)
		framedata = m.readRegisters16(0x0400, 832)
		statusRegister = m.readRegisters16(0x8000, 1)
		cnt += 1
		dataReady = int(statusRegister[0] & 0x0008)
	}

	if cnt > 4 {
		log.Fatal("Too many retries")
	}

	controlRegister := make([]uint16, 0)
	controlRegister = m.readRegisters16(0x800D, 1)

	for i := 0; i < len(framedata); i++ {
		m._frameData[i] = framedata[i]
	}

	m._frameData[832] = controlRegister[0]
	m._frameData[833] = uint16(statusRegister[0] & 0x0001)

	return m._frameData[833]
}

func (m *Mlx90640) GetTa() float64 {
	ptat := float64(m._frameData[800])
	ptatArt := float64(m._frameData[m.pixelCount])
	ptatArt = float64((ptat / float64((ptat*m._alphaPTAT + ptatArt)) * float64(math.Pow(2, 18))))

	vdd := m.GetVdd()
	ta := float64((ptatArt/float64(1+m._kvPTAT*(vdd-3.3)) - float64(m._vPTAT25)))
	ta = ta/m._ktPTAT + 25

	return ta
}

func (m *Mlx90640) GetVdd() float64 {
	vdd := float64(m._frameData[810])
	if vdd > 32767 {
		vdd -= 65536
	}

	resolutionRAM := (m._frameData[832] & 0x0C00) >> 10
	resolutionCorrection := float64(math.Pow(2, float64(m._resolutionEE)) / math.Pow(2, float64(resolutionRAM)))
	vdd = (resolutionCorrection*vdd-float64(m._vdd25))/float64(m._kVdd) + 3.3

	return vdd
}

func (m *Mlx90640) DumpEE() {
	m._eeData = m.readRegisters16(0x2400, 832)
}

func (m *Mlx90640) ExtractVDDParameters() {
	m._kVdd = int16((m._eeData[51] & 0xff00) >> 8)
	if m._kVdd > 127 {
		m._kVdd -= 256
	}

	m._kVdd *= 32
	m._vdd25 = int16(m._eeData[51] & 0x00ff)
	m._vdd25 = (m._vdd25-256)<<5 - 8192
}

func (m *Mlx90640) ExtractPTATParameters() {
	m._kvPTAT = float64((m._eeData[50] & 0xFC00) >> 10)
	if m._kvPTAT > 31 {
		m._kvPTAT -= 64
	}

	m._kvPTAT /= 4096
	m._ktPTAT = float64(m._eeData[50] & 0x03FF)
	if m._ktPTAT > 511 {
		m._ktPTAT -= 1024
	}

	m._ktPTAT /= 8
	m._vPTAT25 = int16(m._eeData[49])
	m._alphaPTAT = float64(float64((m._eeData[16]&0xF000))/float64(math.Pow(2, 14)) + 8)
}

func (m *Mlx90640) ExtractGainParameters() {
	m._gainEE = int16(m._eeData[48])
	if m._gainEE > 32767 {
		m._gainEE = int16(int(m._gainEE))
	}
}

func (m *Mlx90640) ExtractTgcParameters() {
	m._tgc = float64(m._eeData[60] & 0x00FF)
	if m._tgc > 127 {
		m._tgc -= 256
	}

	m._tgc /= 32
}

func (m *Mlx90640) ExtractResolutionParameters() {
	m._resolutionEE = byte((m._eeData[56] & 0x3000) >> 12)
}

func (m *Mlx90640) ExtractKsTaParameters() {
	m._ksTa = float64((m._eeData[60] & 0xFF00) >> 8)
	if m._ksTa > 127 {
		m._ksTa -= 256
	}

	m._ksTa /= 8192
}

func (m *Mlx90640) ExtractKsToParameters() {
	step := byte((((m._eeData[63] & 0x3000) >> 12) * 10))
	m._ct[0] = -40
	m._ct[1] = 0
	m._ct[2] = int((m._eeData[63] & 0x00F0) >> 4)
	m._ct[3] = int((m._eeData[63] & 0x0F00) >> 8)
	m._ct[2] *= int(step)
	m._ct[3] = m._ct[2] + m._ct[3]*int(step)

	ksToScale := int((m._eeData[63] & 0x000F) + 8)
	ksToScale = 1 << ksToScale

	m._ksTo[0] = float64(m._eeData[61] & 0x00FF)
	m._ksTo[1] = float64((m._eeData[61] & 0xFF00) >> 8)
	m._ksTo[2] = float64(m._eeData[62] & 0x00FF)
	m._ksTo[3] = float64((m._eeData[62] & 0xFF00) >> 8)

	for i := 0; i < 4; i++ {
		if m._ksTo[i] > 127 {
			m._ksTo[i] -= 256
		}

		m._ksTo[i] /= float64(ksToScale)
	}

	m._ksTo[4] = -0.0002
}
func (m *Mlx90640) ExtractCPParameters() {
	alphaScale := float64((byte)(((m._eeData[32] & 0xF000) >> 12) + 27))

	offsetSP := [2]int16{}
	offsetSP[0] = int16(m._eeData[58] & 0x03FF)
	if offsetSP[0] > 511 {
		offsetSP[0] = int16(offsetSP[0] - 1024)
	}

	offsetSP[1] = int16((m._eeData[58] & 0xFC00) >> 10)
	if offsetSP[1] > 31 {
		offsetSP[1] = int16(offsetSP[1] - 64)
	}

	offsetSP[1] = int16(offsetSP[1] + offsetSP[0])

	alphaSP := [2]float64{}
	alphaSP[0] = float64(m._eeData[57] & 0x03FF)

	if alphaSP[0] > 511 {
		alphaSP[0] = float64(alphaSP[0] - 1024)
	}

	alphaSP[0] = float64(alphaSP[0] / float64(math.Pow(2, float64(alphaScale))))

	alphaSP[1] = float64((m._eeData[57] & 0xFC00) >> 10)
	if alphaSP[1] > 31 {
		alphaSP[1] = alphaSP[1] - 64
	}

	alphaSP[1] = (1 + alphaSP[1]/128) * alphaSP[0]

	cpKta := float64(m._eeData[59] & 0x00FF)
	if cpKta > 127 {
		cpKta = cpKta - 256
	}

	ktaScale1 := float64(((m._eeData[56] & 0x00F0) >> 4) + 8)
	m._cpKta = float64(cpKta / float64(math.Pow(2, float64(ktaScale1))))

	cpKv := float64((m._eeData[59] & 0xFF00) >> 8)
	if cpKv > 127 {
		cpKv = cpKv - 256
	}

	kvScale := float64((m._eeData[56] & 0x0F00) >> 8)
	m._cpKv = float64(cpKv / float64(math.Pow(2, float64(kvScale))))

	m._cpAlpha[0] = alphaSP[0]
	m._cpAlpha[1] = alphaSP[1]
	m._cpOffset[0] = int(offsetSP[0])
	m._cpOffset[1] = int(offsetSP[1])
}

func (m *Mlx90640) ExtractAlphaParameters() {
	accRemScale := byte(m._eeData[32] & 0x000F)
	accColumnScale := byte((m._eeData[32] & 0x00F0) >> 4)
	accRowScale := byte((m._eeData[32] & 0x0F00) >> 8)
	alphaScale := byte(((m._eeData[32] & 0xF000) >> 12) + 30)
	alphaRef := m._eeData[33]

	accRow := make([]int, 24)
	for i := 0; i < 6; i++ {
		p := i * 4
		accRow[p+0] = int(m._eeData[34+i] & 0x000F)
		accRow[p+1] = int(m._eeData[34+i]&0x00F0) >> 4
		accRow[p+2] = int(m._eeData[34+i]&0x0F00) >> 8
		accRow[p+3] = int(m._eeData[34+i]&0xF000) >> 12
	}

	for i := 0; i < 24; i++ {
		if accRow[i] > 7 {
			accRow[i] = accRow[i] - 16
		}
	}

	accColumn := make([]int, 32)
	for i := 0; i < 8; i++ {
		p := i * 4
		accColumn[p+0] = int(m._eeData[40+i] & 0x000F)
		accColumn[p+1] = int(m._eeData[40+i]&0x00F0) >> 4
		accColumn[p+2] = int(m._eeData[40+i]&0x0F00) >> 8
		accColumn[p+3] = int(m._eeData[40+i]&0xF000) >> 12
	}

	for i := 0; i < 32; i++ {
		if accColumn[i] > 7 {
			accColumn[i] = accColumn[i] - 16
		}
	}

	alphaTemp := make([]float64, m.pixelCount)
	for i := 0; i < 24; i++ {
		for j := 0; j < 32; j++ {
			p := 32*i + j
			alphaTemp[p] = float64((m._eeData[64+p] & 0x03F0) >> 4)
			if alphaTemp[p] > 31 {
				alphaTemp[p] = alphaTemp[p] - 64
			}

			alphaTemp[p] = alphaTemp[p] * float64(int(1)<<accRemScale)
			alphaTemp[p] = float64(int(alphaRef)+(accRow[i]<<accRowScale)) + float64(accColumn[j]<<accColumnScale) + alphaTemp[p]
			alphaTemp[p] = float64(alphaTemp[p] / float64(math.Pow(2, float64(alphaScale))))
			alphaTemp[p] = alphaTemp[p] - m._tgc*(m._cpAlpha[0]+m._cpAlpha[1])/2
			alphaTemp[p] = float64(SCALEALPHA / alphaTemp[p])
		}
	}

	temp := alphaTemp[0]
	for i := 1; i < 768; i++ {
		if alphaTemp[i] > temp {
			temp = alphaTemp[i]
		}
	}

	alphaScale = 0
	for temp < 32767.4 {
		temp = temp * 2
		alphaScale = byte(alphaScale + 1)
	}

	for i := 0; i < 768; i++ {
		temp = float64(alphaTemp[i] * float64(math.Pow(2, float64(alphaScale))))
		m._alpha[i] = int(temp + 0.5)
	}

	m._alphaScale = alphaScale
}

func (m *Mlx90640) ExtractOffsetParameters() {
	occRemScale := byte(m._eeData[16] & 0x000F)
	occColumnScale := byte((m._eeData[16] & 0x00F0) >> 4)
	occRowScale := byte((m._eeData[16] & 0x0F00) >> 8)
	offsetRef := int(m._eeData[17])

	if offsetRef > 32767 {
		offsetRef = int(int16(offsetRef))
	}

	occRow := [24]int{}
	for i := 0; i < 6; i++ {
		p := i * 4
		occRow[p+0] = int(m._eeData[18+i] & 0x000F)
		occRow[p+1] = int(m._eeData[18+i]&0x00F0) >> 4
		occRow[p+2] = int(m._eeData[18+i]&0x0F00) >> 8
		occRow[p+3] = int(m._eeData[18+i]&0xF000) >> 12
	}

	for i := 0; i < 24; i++ {
		if occRow[i] > 7 {
			occRow[i] = occRow[i] - 16
		}
	}

	occColumn := [32]int{}
	for i := 0; i < 8; i++ {
		p := i * 4
		occColumn[p+0] = int(m._eeData[24+i] & 0x000F)
		occColumn[p+1] = int(m._eeData[24+i]&0x00F0) >> 4
		occColumn[p+2] = int(m._eeData[24+i]&0x0F00) >> 8
		occColumn[p+3] = int(m._eeData[24+i]&0xF000) >> 12
	}

	for i := 0; i < 32; i++ {
		if occColumn[i] > 7 {
			occColumn[i] = occColumn[i] - 16
		}
	}

	for i := 0; i < 24; i++ {
		for j := 0; j < 32; j++ {
			p := 32*i + j
			m._offset[p] = int(m._eeData[64+p]&0xFC00) >> 10
			if m._offset[p] > 31 {
				m._offset[p] = m._offset[p] - 64
			}

			m._offset[p] = m._offset[p] * (1 << occRemScale)
			m._offset[p] = (int(offsetRef) + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + m._offset[p])
		}
	}
}

func (m *Mlx90640) ExtractKtaPixelParameters() {
	ktaRC := [4]int8{}
	ktaRC[0] = int8((m._eeData[54] & 0xFF00) >> 8)
	ktaRC[1] = int8((m._eeData[55] & 0xFF00) >> 8)
	ktaRC[2] = int8(m._eeData[54] & 0x00FF)
	ktaRC[3] = int8(m._eeData[55] & 0x00FF)

	ktaScale1 := byte(((m._eeData[56] & 0x00F0) >> 4) + 8)
	ktaScale2 := byte(m._eeData[56] & 0x000F)

	ktaTemp := make([]float64, m.pixelCount)
	for i := 0; i < 24; i++ {
		for j := 0; j < 32; j++ {
			p := 32*i + j
			split := byte(2*(p/32-(p/64)*2) + p%2)
			ktaTemp[p] = float64((m._eeData[64+p] & 0x000E) >> 1)

			if ktaTemp[p] > 3 {
				ktaTemp[p] = ktaTemp[p] - 8
			}

			ktaTemp[p] = ktaTemp[p] * float64((int(1) << ktaScale2))
			ktaTemp[p] = float64(ktaRC[split]) + float64(ktaTemp[p])
			ktaTemp[p] = float64(ktaTemp[p] / float64(math.Pow(2, float64(ktaScale1))))
		}
	}

	temp := math.Abs(float64(ktaTemp[0]))
	for i := 1; i < 768; i++ {
		if math.Abs(float64(ktaTemp[i])) > temp {
			temp = math.Abs(float64(ktaTemp[i]))
		}
	}

	ktaScale1 = 0
	for temp < 63.4 {
		temp = temp * 2
		ktaScale1 = byte(ktaScale1 + 1)
	}

	for i := 0; i < 768; i++ {
		temp := float64((ktaTemp[i] * float64(math.Pow(2, float64(ktaScale1)))))
		if temp < 0 {
			m._kta[i] = (byte)(temp - 0.5)
		} else {
			m._kta[i] = (byte)(temp + 0.5)
		}
	}

	m._ktaScale = ktaScale1
}

func (m *Mlx90640) ExtractKvPixelParameters() {
	kvRoco := int8((m._eeData[52] & 0xF000) >> 12)
	if kvRoco > 7 {
		kvRoco = int8(kvRoco - 16)
	}

	kvt := [4]int8{}
	kvt[0] = kvRoco

	kvReco := int8((m._eeData[52] & 0x0F00) >> 8)
	if kvReco > 7 {
		kvReco = int8(kvReco - 16)
	}

	kvt[2] = kvReco

	kvRoce := int8((m._eeData[52] & 0x00F0) >> 4)
	if kvRoce > 7 {
		kvRoce = int8(kvRoce - 16)
	}

	kvt[1] = kvRoce

	kvRece := int8(m._eeData[52] & 0x000F)
	if kvRece > 7 {
		kvRece = int8(kvRece - 16)
	}

	kvt[3] = kvRece

	kvScale := byte((m._eeData[56] & 0x0F00) >> 8)

	kvtemp := make([]float64, m.pixelCount)
	for i := 0; i < 24; i++ {
		for j := 0; j < 32; j++ {
			p := 32*i + j
			split := byte(2*(p/32-(p/64)*2) + p%2)
			kvtemp[p] = float64(kvt[split])
			kvtemp[p] = float64(kvtemp[p] / float64(math.Pow(2, float64(kvScale))))
		}
	}

	temp := math.Abs(float64(kvtemp[0]))
	for i := 1; i < 768; i++ {
		if math.Abs(float64(kvtemp[i])) > temp {
			temp = math.Abs(float64(kvtemp[i]))
		}
	}

	kvScale = 0
	for temp < 63.4 {
		temp = temp * 2
		kvScale = byte(kvScale + 1)
	}

	for i := 0; i < 768; i++ {
		temp := float64(kvtemp[i] * float64(math.Pow(2, float64(kvScale))))
		if temp < 0 {
			m._kv[i] = int8(temp - 0.5)
		} else {
			m._kv[i] = int8(temp + 0.5)
		}
	}

	m._kvScale = kvScale
}

func (m *Mlx90640) ExtractCILCParameters() {
	calibrationModeEE := byte((m._eeData[10] & 0x0800) >> 4)
	calibrationModeEE = byte(calibrationModeEE ^ 0x80)

	ilChessC := [3]float64{}
	ilChessC[0] = float64(m._eeData[53] & 0x003F)
	if ilChessC[0] > 31 {
		ilChessC[0] = ilChessC[0] - 64
	}

	ilChessC[0] = ilChessC[0] / 16.0

	ilChessC[1] = float64(m._eeData[53] & 0x07C0 >> 6)
	if ilChessC[1] > 15 {
		ilChessC[1] = ilChessC[1] - 32
	}

	ilChessC[1] = ilChessC[1] / 2.0

	ilChessC[2] = float64(m._eeData[53] & 0xF800 >> 11)
	if ilChessC[2] > 15 {
		ilChessC[2] = ilChessC[2] - 32
	}

	ilChessC[2] = ilChessC[2] / 8.0

	m._calibrationModeEE = int(calibrationModeEE)
	m._ilChessC[0] = ilChessC[0]
	m._ilChessC[1] = ilChessC[1]
	m._ilChessC[2] = ilChessC[2]
}

func (m *Mlx90640) ExtractDeviatingPixels() int {
	var pixCnt uint16 = 0
	for pixCnt = 0; pixCnt < 5; pixCnt++ {
		m._brokenPixels[pixCnt] = 0xFFFF
		m._outlierPixels[pixCnt] = 0xFFFF
	}

	var brokenPixCnt uint16 = 0
	var outlierPixCnt uint16 = 0
	for pixCnt < 768 && brokenPixCnt < 5 && outlierPixCnt < 5 {
		if m._eeData[pixCnt+64] == 0 {
			m._brokenPixels[brokenPixCnt] = pixCnt
			brokenPixCnt = uint16(brokenPixCnt + 1)
		} else if (m._eeData[pixCnt+64] & 0x0001) != 0 {
			m._outlierPixels[outlierPixCnt] = pixCnt
			outlierPixCnt = uint16(outlierPixCnt + 1)
		}

		pixCnt = uint16(pixCnt + 1)
	}

	warn := 0
	if brokenPixCnt > 4 {
		warn = -3
	} else if outlierPixCnt > 4 {
		warn = -4
	} else if (brokenPixCnt + outlierPixCnt) > 4 {
		warn = -5
	} else {
		for pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++ {
			for i := pixCnt + 1; i < brokenPixCnt; i++ {
				warn = CheckAdjacentPixels(m._brokenPixels[pixCnt], m._brokenPixels[i])
				if warn != 0 {
					return warn
				}
			}
		}

		for pixCnt = 0; pixCnt < outlierPixCnt; pixCnt++ {
			for i := pixCnt + 1; i < outlierPixCnt; i++ {
				warn = CheckAdjacentPixels(m._outlierPixels[pixCnt], m._outlierPixels[i])
				if warn != 0 {
					return warn
				}
			}
		}

		for pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++ {
			for i := uint16(0); i < outlierPixCnt; i++ {
				warn = CheckAdjacentPixels(m._brokenPixels[pixCnt], m._outlierPixels[i])
				if warn != 0 {
					return warn
				}
			}
		}
	}

	return warn
}

func CheckAdjacentPixels(pix1 uint16, pix2 uint16) int {
	pixPosDif := int(pix1 - pix2)
	if pixPosDif > -34 && pixPosDif < -30 {
		return -6
	}

	if pixPosDif > -2 && pixPosDif < 2 {
		return -6
	}

	if pixPosDif > 30 && pixPosDif < 34 {
		return -6
	}

	return 0
}

func (m *Mlx90640) readRegisters16(addr int, length int) []uint16 {
	read := make([]byte, length*2)
	dataOut := make([]uint16, length)

	buf := make([]byte, 2)
	buf[0] = byte(addr >> 8)
	buf[1] = byte(addr & 0x00FF)

	if err := m.i2c.Tx(buf, read); err != nil {
		log.Fatal(err)
	}

	for cnt := 0; cnt < length; cnt++ {
		i := uint16(cnt << 1)
		dataOut[cnt] = binary.BigEndian.Uint16([]byte{read[i], read[i+1]})
	}

	return dataOut
}

func (m *Mlx90640) write16(writeAddress int, data int) {
	cmd := make([]byte, 4)

	cmd[0] = byte(writeAddress >> 8)
	cmd[1] = byte(writeAddress & 0xFF)
	cmd[2] = byte(data >> 8)
	cmd[3] = byte(data & 0xFF)

	_, err := m.i2c.Write(cmd)
	if err != nil {
		log.Fatal(err)
	}
}
