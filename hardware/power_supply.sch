EESchema Schematic File Version 4
LIBS:datalogger-cache
LIBS:RTRatuj-cache
LIBS:power_supply-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title "field datalogger"
Date "2019-06-06"
Rev "v. 1.1"
Comp "grzegorz@sapijaszko.net"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:LED D3
U 1 1 5CA68559
P 2600 1750
F 0 "D3" V 2650 1650 50  0000 R CNN
F 1 "red" V 2550 1650 50  0000 R CNN
F 2 "LED_SMD:LED_1210_3225Metric_Pad1.42x2.65mm_HandSolder" H 2600 1750 50  0001 C CNN
F 3 "~" H 2600 1750 50  0001 C CNN
	1    2600 1750
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5CA6855F
P 2200 1750
F 0 "D2" V 2250 1650 50  0000 R CNN
F 1 "green" V 2150 1650 50  0000 R CNN
F 2 "LED_SMD:LED_1210_3225Metric_Pad1.42x2.65mm_HandSolder" H 2200 1750 50  0001 C CNN
F 3 "~" H 2200 1750 50  0001 C CNN
	1    2200 1750
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R10
U 1 1 5CA68565
P 2900 2100
F 0 "R10" V 3000 2100 50  0000 C CNN
F 1 "1k" V 3100 2100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2830 2100 50  0001 C CNN
F 3 "~" H 2900 2100 50  0001 C CNN
	1    2900 2100
	0    1    1    0   
$EndComp
$Comp
L Device:R R12
U 1 1 5CA6856B
P 2600 2400
F 0 "R12" V 2700 2400 50  0000 C CNN
F 1 "1k" V 2800 2400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2530 2400 50  0001 C CNN
F 3 "~" H 2600 2400 50  0001 C CNN
	1    2600 2400
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 2400 3050 2400
Wire Wire Line
	2450 2400 2200 2400
Wire Wire Line
	2200 2400 2200 1900
Wire Wire Line
	2750 2100 2600 2100
Wire Wire Line
	2600 2100 2600 1900
Wire Wire Line
	3050 1600 2600 1600
Wire Wire Line
	2600 1600 2600 1500
Connection ~ 2600 1600
Wire Wire Line
	3050 1500 2600 1500
Connection ~ 2600 1500
Wire Wire Line
	2600 1500 2600 1300
Wire Wire Line
	2000 1300 2200 1300
Connection ~ 2600 1300
Wire Wire Line
	2200 1600 2200 1300
Connection ~ 2200 1300
Wire Wire Line
	2200 1300 2600 1300
$Comp
L Device:C C19
U 1 1 5CA68581
P 2000 2500
F 0 "C19" H 2115 2546 50  0000 L CNN
F 1 "10uF" H 2115 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 2038 2350 50  0001 C CNN
F 3 "~" H 2000 2500 50  0001 C CNN
	1    2000 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 1300 2000 2350
Wire Wire Line
	2000 2750 2000 2650
$Comp
L Device:R R14
U 1 1 5CA68589
P 4050 2550
F 0 "R14" H 4120 2596 50  0000 L CNN
F 1 "1k2" H 4120 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 3980 2550 50  0001 C CNN
F 3 "~" H 4050 2550 50  0001 C CNN
	1    4050 2550
	1    0    0    -1  
$EndComp
$Comp
L datalogger-rescue:TP4056-kicad_libraries U6
U 1 1 5CA6858F
P 3550 1750
AR Path="/5CA6858F" Ref="U6"  Part="1" 
AR Path="/5C346344/5CA6858F" Ref="U6"  Part="1" 
F 0 "U6" H 3550 2400 60  0000 C CNN
F 1 "TP4056" H 3550 2300 60  0000 C CNN
F 2 "Package_SO:HSOP-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.1mm_ThermalVias" H 3550 1750 60  0001 C CNN
F 3 "" H 3550 1750 60  0000 C CNN
	1    3550 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 2750 3550 2750
Connection ~ 3550 2750
Wire Wire Line
	3550 2750 3650 2750
Connection ~ 3650 2750
Text Notes 3150 900  0    50   ~ 0
CHARGER
$Comp
L Connector:USB_B_Mini J6
U 1 1 5CA685D6
P 1250 1850
F 0 "J6" H 1307 2317 50  0000 C CNN
F 1 "USB_B_Mini" H 1307 2226 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Wuerth_629105150521" H 1400 1800 50  0001 C CNN
F 3 "~" H 1400 1800 50  0001 C CNN
	1    1250 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 2250 1250 2750
Wire Wire Line
	1250 2750 2000 2750
Connection ~ 2000 2750
Wire Wire Line
	1150 2250 1150 2750
Wire Wire Line
	1150 2750 1250 2750
Connection ~ 1250 2750
Wire Wire Line
	1550 1650 1600 1650
Wire Wire Line
	1600 1650 1600 1300
Wire Notes Line
	7150 750  10750 750 
Wire Notes Line
	10750 3050 7150 3050
Text Notes 8700 900  0    50   ~ 0
+3.3V CONVERTER
Wire Wire Line
	1600 1300 1700 1300
Connection ~ 2000 1300
$Comp
L Device:R R8
U 1 1 5CA685EB
P 1850 1300
F 0 "R8" H 1920 1346 50  0000 L CNN
F 1 "0R4" H 1920 1255 50  0000 L CNN
F 2 "Resistor_SMD:R_1210_3225Metric_Pad1.42x2.65mm_HandSolder" V 1780 1300 50  0001 C CNN
F 3 "~" H 1850 1300 50  0001 C CNN
	1    1850 1300
	0    1    1    0   
$EndComp
NoConn ~ 1550 2050
Wire Wire Line
	2600 1300 4650 1300
Connection ~ 4300 2750
Connection ~ 4650 2750
$Comp
L Device:R R15
U 1 1 5CA685F9
P 4650 2550
F 0 "R15" H 4720 2596 50  0000 L CNN
F 1 "10k48" H 4720 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4580 2550 50  0001 C CNN
F 3 "~" H 4650 2550 50  0001 C CNN
	1    4650 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2300 4650 2250
Wire Wire Line
	4650 2750 5150 2750
Wire Wire Line
	4650 2300 5150 2300
$Comp
L Device:Thermistor_NTC TH1
U 1 1 5CA68604
P 5150 2550
F 0 "TH1" H 5248 2596 50  0000 L CNN
F 1 "10k (25ᵒ)" H 5248 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_1218_3246Metric_Pad1.22x4.75mm_HandSolder" H 5150 2600 50  0001 C CNN
F 3 "~" H 5150 2600 50  0001 C CNN
	1    5150 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2750 4650 2750
Connection ~ 4650 2300
Wire Wire Line
	4050 2300 4650 2300
$Comp
L Device:R R11
U 1 1 5CA6860D
P 4650 2100
F 0 "R11" H 4720 2146 50  0000 L CNN
F 1 "2k7" H 4720 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 4580 2100 50  0001 C CNN
F 3 "~" H 4650 2100 50  0001 C CNN
	1    4650 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 1300 4650 1950
Wire Wire Line
	4050 1500 4300 1500
Wire Wire Line
	4300 1550 4300 1500
Connection ~ 4300 1500
$Comp
L Connector:Conn_01x02_Female J7
U 1 1 5CA68617
P 5900 1950
F 0 "J7" H 5928 1926 50  0000 L CNN
F 1 "BAT" H 5928 1835 50  0000 L CNN
F 2 "Connector_JST:JST_XH_S02B-XH-A_1x02_P2.50mm_Horizontal" H 5900 1950 50  0001 C CNN
F 3 "~" H 5900 1950 50  0001 C CNN
	1    5900 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C17
U 1 1 5CA6861D
P 4300 1700
F 0 "C17" H 4415 1746 50  0000 L CNN
F 1 "10uF" H 4415 1655 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 4338 1550 50  0001 C CNN
F 3 "~" H 4300 1700 50  0001 C CNN
	1    4300 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1850 4300 2750
Wire Wire Line
	4300 1500 5700 1500
Text GLabel 5800 1500 2    50   Input ~ 0
BAT+
$Comp
L Transistor_FET:2N7002 Q?
U 1 1 5CA291C4
P 7000 4700
AR Path="/5CA291C4" Ref="Q?"  Part="1" 
AR Path="/5C346344/5CA291C4" Ref="Q3"  Part="1" 
F 0 "Q3" H 7206 4746 50  0000 L CNN
F 1 "2N7002" H 7206 4655 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TSOT-23" H 7200 4625 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7002.pdf" H 7000 4700 50  0001 L CNN
	1    7000 4700
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:Si2319CDS Q?
U 1 1 5CA291CA
P 7450 4400
AR Path="/5CA291CA" Ref="Q?"  Part="1" 
AR Path="/5C346344/5CA291CA" Ref="Q2"  Part="1" 
F 0 "Q2" H 7656 4354 50  0000 L CNN
F 1 "Si2309DS" H 7656 4445 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7650 4325 50  0001 L CIN
F 3 "http://www.vishay.com/docs/66709/si2319cd.pdf" H 7450 4400 50  0001 L CNN
	1    7450 4400
	1    0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 5CA291D1
P 7550 4950
AR Path="/5CA291D1" Ref="R?"  Part="1" 
AR Path="/5C346344/5CA291D1" Ref="R21"  Part="1" 
F 0 "R21" H 7620 4996 50  0000 L CNN
F 1 "330k" H 7620 4905 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7480 4950 50  0001 C CNN
F 3 "~" H 7550 4950 50  0001 C CNN
	1    7550 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5CA291D7
P 7550 5450
AR Path="/5CA291D7" Ref="R?"  Part="1" 
AR Path="/5C346344/5CA291D7" Ref="R25"  Part="1" 
F 0 "R25" H 7620 5496 50  0000 L CNN
F 1 "100k" H 7620 5405 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7480 5450 50  0001 C CNN
F 3 "~" H 7550 5450 50  0001 C CNN
	1    7550 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5CA291DD
P 6700 4850
AR Path="/5CA291DD" Ref="R?"  Part="1" 
AR Path="/5C346344/5CA291DD" Ref="R19"  Part="1" 
F 0 "R19" H 6770 4896 50  0000 L CNN
F 1 "100k" H 6770 4805 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 6630 4850 50  0001 C CNN
F 3 "~" H 6700 4850 50  0001 C CNN
	1    6700 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5CA291E3
P 7100 4150
AR Path="/5CA291E3" Ref="R?"  Part="1" 
AR Path="/5C346344/5CA291E3" Ref="R16"  Part="1" 
F 0 "R16" H 7170 4196 50  0000 L CNN
F 1 "10k" H 7170 4105 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7030 4150 50  0001 C CNN
F 3 "~" H 7100 4150 50  0001 C CNN
	1    7100 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 4000 7550 4200
Wire Wire Line
	7550 4600 7550 4800
$Comp
L Device:C C?
U 1 1 5CA291EB
P 7100 5450
AR Path="/5CA291EB" Ref="C?"  Part="1" 
AR Path="/5C346344/5CA291EB" Ref="C27"  Part="1" 
F 0 "C27" H 7215 5496 50  0000 L CNN
F 1 "100nF" H 7215 5405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7138 5300 50  0001 C CNN
F 3 "~" H 7100 5450 50  0001 C CNN
	1    7100 5450
	1    0    0    -1  
$EndComp
Connection ~ 7550 5300
Wire Wire Line
	7550 5300 7100 5300
Wire Wire Line
	6300 4000 7100 4000
Connection ~ 7100 4000
Wire Wire Line
	7100 4000 7550 4000
Wire Wire Line
	7550 5100 7550 5300
$Comp
L power:GND #PWR?
U 1 1 5CA291F7
P 7100 5000
AR Path="/5CA291F7" Ref="#PWR?"  Part="1" 
AR Path="/5C346344/5CA291F7" Ref="#PWR049"  Part="1" 
F 0 "#PWR049" H 7100 4750 50  0001 C CNN
F 1 "GND" H 7105 4827 50  0000 C CNN
F 2 "" H 7100 5000 50  0001 C CNN
F 3 "" H 7100 5000 50  0001 C CNN
	1    7100 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CA291FD
P 7550 5600
AR Path="/5CA291FD" Ref="#PWR?"  Part="1" 
AR Path="/5C346344/5CA291FD" Ref="#PWR051"  Part="1" 
F 0 "#PWR051" H 7550 5350 50  0001 C CNN
F 1 "GND" H 7555 5427 50  0000 C CNN
F 2 "" H 7550 5600 50  0001 C CNN
F 3 "" H 7550 5600 50  0001 C CNN
	1    7550 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CA29203
P 7100 5600
AR Path="/5CA29203" Ref="#PWR?"  Part="1" 
AR Path="/5C346344/5CA29203" Ref="#PWR050"  Part="1" 
F 0 "#PWR050" H 7100 5350 50  0001 C CNN
F 1 "GND" H 7105 5427 50  0000 C CNN
F 2 "" H 7100 5600 50  0001 C CNN
F 3 "" H 7100 5600 50  0001 C CNN
	1    7100 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 5000 7100 4900
$Comp
L power:GND #PWR?
U 1 1 5CA2920A
P 6700 5000
AR Path="/5CA2920A" Ref="#PWR?"  Part="1" 
AR Path="/5C346344/5CA2920A" Ref="#PWR048"  Part="1" 
F 0 "#PWR048" H 6700 4750 50  0001 C CNN
F 1 "GND" H 6705 4827 50  0000 C CNN
F 2 "" H 6700 5000 50  0001 C CNN
F 3 "" H 6700 5000 50  0001 C CNN
	1    6700 5000
	1    0    0    -1  
$EndComp
Text GLabel 6300 4000 0    50   Input ~ 0
BAT+
Text Notes 6500 3750 0    50   ~ 0
BATTERY MEASUREMENT
Wire Wire Line
	7100 4300 7100 4400
Wire Wire Line
	7250 4400 7100 4400
Connection ~ 7100 4400
Wire Wire Line
	7100 4400 7100 4500
Connection ~ 6700 4700
Wire Wire Line
	6700 4700 6800 4700
Wire Wire Line
	6300 4700 6700 4700
Connection ~ 7100 5300
Wire Wire Line
	7100 5300 6300 5300
Wire Notes Line
	5850 3600 8100 3600
Wire Notes Line
	5850 5900 8100 5900
Text GLabel 6300 4700 0    50   Input ~ 0
ENABLE
Text GLabel 6300 5300 0    50   Input ~ 0
ADC
Text GLabel 1550 1850 2    50   Input ~ 0
UD+
Text GLabel 1550 1950 2    50   Input ~ 0
UD-
$Comp
L Device:C C20
U 1 1 5CAF89C3
P 1750 4800
F 0 "C20" H 1865 4846 50  0000 L CNN
F 1 "10uF" H 1865 4755 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 1788 4650 50  0001 C CNN
F 3 "~" H 1750 4800 50  0001 C CNN
	1    1750 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 4650 1750 4650
Connection ~ 1750 4650
Wire Wire Line
	1750 4650 1400 4650
$Comp
L Connector:Conn_01x02_Female J8
U 1 1 5CAFC5C9
P 1200 4650
F 0 "J8" H 1092 4835 50  0000 C CNN
F 1 "PhotoCell" H 1092 4744 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 1200 4650 50  0001 C CNN
F 3 "~" H 1200 4650 50  0001 C CNN
	1    1200 4650
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR046
U 1 1 5CB00A4F
P 1400 4750
F 0 "#PWR046" H 1400 4500 50  0001 C CNN
F 1 "GND" H 1405 4577 50  0000 C CNN
F 2 "" H 1400 4750 50  0001 C CNN
F 3 "" H 1400 4750 50  0001 C CNN
	1    1400 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR047
U 1 1 5CB010BD
P 1750 4950
F 0 "#PWR047" H 1750 4700 50  0001 C CNN
F 1 "GND" H 1755 4777 50  0000 C CNN
F 2 "" H 1750 4950 50  0001 C CNN
F 3 "" H 1750 4950 50  0001 C CNN
	1    1750 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R23
U 1 1 5CB01965
P 1400 5450
F 0 "R23" H 1470 5496 50  0000 L CNN
F 1 "40k2" H 1470 5405 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 1330 5450 50  0001 C CNN
F 3 "~" H 1400 5450 50  0001 C CNN
	1    1400 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 5250 1400 5250
Wire Wire Line
	1400 5250 1400 5300
$Comp
L Device:C C25
U 1 1 5CB063DF
P 1750 5700
F 0 "C25" H 1865 5746 50  0000 L CNN
F 1 "1uF" H 1865 5655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1788 5550 50  0001 C CNN
F 3 "~" H 1750 5700 50  0001 C CNN
	1    1750 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 5600 1400 5900
Wire Wire Line
	1400 5900 1750 5900
Wire Wire Line
	1750 5850 1750 5900
Wire Wire Line
	1750 5550 1950 5550
Wire Wire Line
	2950 5550 3000 5550
Wire Wire Line
	3000 5550 3000 5900
$Comp
L power:GND #PWR052
U 1 1 5CB1609E
P 2450 5900
F 0 "#PWR052" H 2450 5650 50  0001 C CNN
F 1 "GND" H 2455 5727 50  0000 C CNN
F 2 "" H 2450 5900 50  0001 C CNN
F 3 "" H 2450 5900 50  0001 C CNN
	1    2450 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C26
U 1 1 5CB169A5
P 3150 5700
F 0 "C26" H 3250 5700 50  0000 L CNN
F 1 "4.7uF" H 3200 5600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3188 5550 50  0001 C CNN
F 3 "~" H 3150 5700 50  0001 C CNN
	1    3150 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 5400 3150 5400
Wire Wire Line
	3150 5400 3150 5550
Wire Wire Line
	3150 5850 3150 5900
Wire Wire Line
	3150 5900 3000 5900
$Comp
L Device:R R18
U 1 1 5CB1FEE6
P 3450 4950
F 0 "R18" H 3520 4996 50  0000 L CNN
F 1 "1.02M" H 3520 4905 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 3380 4950 50  0001 C CNN
F 3 "~" H 3450 4950 50  0001 C CNN
	1    3450 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R22
U 1 1 5CB21020
P 3450 5250
F 0 "R22" H 3520 5296 50  0000 L CNN
F 1 "332k" H 3520 5205 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 3380 5250 50  0001 C CNN
F 3 "~" H 3450 5250 50  0001 C CNN
	1    3450 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 5400 3450 5900
Wire Wire Line
	3450 5900 3150 5900
Connection ~ 3150 5900
Wire Wire Line
	2950 5100 3450 5100
Connection ~ 3450 5100
Wire Wire Line
	2950 4800 3450 4800
Connection ~ 3450 4800
Wire Wire Line
	3450 4800 3750 4800
$Comp
L Device:C C24
U 1 1 5CB3D153
P 3750 5550
F 0 "C24" H 3865 5596 50  0000 L CNN
F 1 "10uF" H 3865 5505 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3788 5400 50  0001 C CNN
F 3 "~" H 3750 5550 50  0001 C CNN
	1    3750 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 4800 3750 5400
Connection ~ 3750 4800
Wire Wire Line
	3750 4800 3900 4800
Wire Wire Line
	3750 5700 3750 5900
Wire Wire Line
	3750 5900 3450 5900
Connection ~ 3450 5900
$Comp
L Device:L L2
U 1 1 5CB479B2
P 2450 4250
F 0 "L2" V 2640 4250 50  0000 C CNN
F 1 "10uH" V 2549 4250 50  0000 C CNN
F 2 "Inductor_SMD:L_7.3x7.3_H4.5" H 2450 4250 50  0001 C CNN
F 3 "~" H 2450 4250 50  0001 C CNN
	1    2450 4250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 4650 1750 4250
Wire Wire Line
	1750 4250 2300 4250
Wire Wire Line
	2600 4250 3100 4250
Wire Wire Line
	3100 4250 3100 4650
Wire Wire Line
	3100 4650 2950 4650
$Comp
L power:GND #PWR044
U 1 1 5CA68625
P 4650 2750
F 0 "#PWR044" H 4650 2500 50  0001 C CNN
F 1 "GND" H 4655 2577 50  0000 C CNN
F 2 "" H 4650 2750 50  0001 C CNN
F 3 "" H 4650 2750 50  0001 C CNN
	1    4650 2750
	1    0    0    -1  
$EndComp
Wire Notes Line
	900  750  900  3050
Wire Notes Line
	6150 3050 6150 750 
Wire Notes Line
	7150 3050 7150 750 
Wire Notes Line
	10750 750  10750 3050
Wire Notes Line
	8100 5900 8100 3600
Wire Notes Line
	5850 3600 5850 5900
Text GLabel 3900 4800 2    50   Input ~ 0
BAT+
Wire Notes Line
	4300 6150 900  6150
Wire Notes Line
	900  6150 900  3450
Wire Notes Line
	900  3450 4300 3450
Wire Notes Line
	4300 3450 4300 6150
Text Notes 2850 3600 2    50   ~ 0
PHOTOVOLTAIC CHARGER
Text Notes 1800 6350 0    50   ~ 0
LT3105 w obudowie 12 nóżek! Poprawić
$Comp
L kicad_libraries:LTC3105 U8
U 1 1 5CC770DE
P 2450 5050
F 0 "U8" H 2450 5720 50  0000 C CNN
F 1 "LTC3105" H 2450 5629 50  0000 C CNN
F 2 "Package_SO:MSOP-12_3x4mm_P0.65mm" H 2100 4100 50  0001 L BNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/3105fb.pdf" H 3000 5100 50  0001 L BNN
	1    2450 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 5900 2350 5900
Connection ~ 1750 5900
Connection ~ 3000 5900
Connection ~ 2350 5900
Wire Wire Line
	2350 5900 2450 5900
Connection ~ 2450 5900
Wire Wire Line
	2450 5900 3000 5900
$Comp
L Device:C C18
U 1 1 5CFA2BF9
P 7800 1650
F 0 "C18" H 7915 1696 50  0000 L CNN
F 1 "1uF" H 7915 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7838 1500 50  0001 C CNN
F 3 "~" H 7800 1650 50  0001 C CNN
	1    7800 1650
	1    0    0    -1  
$EndComp
Connection ~ 7800 1500
Wire Wire Line
	7800 1800 7800 2000
Text GLabel 7600 1500 0    50   Input ~ 0
BAT+
Wire Wire Line
	7600 1500 7800 1500
Wire Wire Line
	7800 2000 8200 2000
Wire Wire Line
	8200 2000 8900 2000
Connection ~ 8200 2000
Wire Wire Line
	8200 1700 8400 1700
Wire Wire Line
	8200 1500 8400 1500
Connection ~ 8200 1500
Connection ~ 8200 1700
Wire Wire Line
	8200 1700 8200 1500
Wire Wire Line
	9600 2000 9950 2000
Connection ~ 9600 2000
Wire Wire Line
	8900 2000 9600 2000
Connection ~ 8900 2000
Connection ~ 9950 2000
Wire Wire Line
	9950 1800 9950 2000
Wire Wire Line
	9950 1500 10200 1500
Connection ~ 9950 1500
Wire Wire Line
	9400 1500 9950 1500
Wire Wire Line
	9400 1700 9600 1700
Wire Wire Line
	7800 1500 8200 1500
$Comp
L power:GND #PWR0102
U 1 1 5CFA2BFF
P 9950 2000
F 0 "#PWR0102" H 9950 1750 50  0001 C CNN
F 1 "GND" H 9955 1827 50  0000 C CNN
F 2 "" H 9950 2000 50  0001 C CNN
F 3 "" H 9950 2000 50  0001 C CNN
	1    9950 2000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C21
U 1 1 5CFA2BF3
P 9950 1650
F 0 "C21" H 10065 1696 50  0000 L CNN
F 1 "1uF" H 10065 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9988 1500 50  0001 C CNN
F 3 "~" H 9950 1650 50  0001 C CNN
	1    9950 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C22
U 1 1 5CFA2BED
P 9600 1850
F 0 "C22" H 9715 1896 50  0000 L CNN
F 1 "22nF" H 9715 1805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9638 1700 50  0001 C CNN
F 3 "~" H 9600 1850 50  0001 C CNN
	1    9600 1850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5CFA2BE7
P 8200 1850
F 0 "R9" H 8270 1896 50  0000 L CNN
F 1 "100k" H 8270 1805 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 8130 1850 50  0001 C CNN
F 3 "~" H 8200 1850 50  0001 C CNN
	1    8200 1850
	1    0    0    -1  
$EndComp
$Comp
L mySensors_door_switch-rescue:RT9193-33GB-kicad_libraries U?
U 1 1 5CFA2BE1
P 8900 1600
AR Path="/5CFA2BE1" Ref="U?"  Part="1" 
AR Path="/5C384508/5CFA2BE1" Ref="U?"  Part="1" 
AR Path="/5C346344/5CFA2BE1" Ref="U5"  Part="1" 
F 0 "U5" H 8900 1967 50  0000 C CNN
F 1 "RT9193-33GB" H 8900 1876 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5_HandSoldering" H 8950 1300 50  0001 L BNN
F 3 "https://www.richtek.com/assets/product_file/RT9193/DS9193-16.pdf" H 8950 1200 50  0001 L BNN
	1    8900 1600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0103
U 1 1 5CFA2BDB
P 10200 1500
F 0 "#PWR0103" H 10200 1350 50  0001 C CNN
F 1 "+3.3V" V 10215 1628 50  0000 L CNN
F 2 "" H 10200 1500 50  0001 C CNN
F 3 "" H 10200 1500 50  0001 C CNN
	1    10200 1500
	0    1    1    0   
$EndComp
Wire Notes Line
	900  3050 6150 3050
Wire Notes Line
	6150 750  900  750 
Wire Wire Line
	4650 2300 4650 2400
Wire Wire Line
	3650 2750 4050 2750
Wire Wire Line
	5150 2300 5150 2400
Wire Wire Line
	5150 2750 5150 2700
Wire Wire Line
	5700 1950 5700 1500
Connection ~ 5700 1500
Wire Wire Line
	5700 1500 5800 1500
Wire Wire Line
	5700 2050 5700 2750
Wire Wire Line
	5700 2750 5150 2750
Connection ~ 5150 2750
Wire Wire Line
	4650 2700 4650 2750
Wire Wire Line
	4050 2700 4050 2750
Connection ~ 4050 2750
Wire Wire Line
	4050 2750 4300 2750
$EndSCHEMATC
