EESchema Schematic File Version 4
LIBS:LoRa_Balloon-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L dtusat-capacitors:C_0402_100nF_CID2080 C?
U 1 1 5DEFBEE1
P 900 1700
F 0 "C?" V 964 1736 35  0000 R CNN
F 1 "C_0402_100nF_CID2080" V 1025 1700 35  0001 C CNN
F 2 "dtusat:C0402" V 1225 1700 35  0001 C CNN
F 3 "" H 900 1700 30  0000 C CNN
F 4 "100nF" V 830 1784 35  0000 R CNN "Component value"
F 5 "2080" V 1075 1700 35  0001 C CNN "CID"
F 6 "X7R" V 1125 1700 35  0001 C CNN "Dielectric"
F 7 "10%" V 1175 1700 35  0001 C CNN "Tolerance"
	1    900  1700
	-1   0    0    -1  
$EndComp
$Comp
L dtusat-capacitors:C_0402_100nF_CID2080 C?
U 1 1 5DEFE1CA
P 1100 1700
F 0 "C?" V 1030 1664 35  0000 L CNN
F 1 "C_0402_100nF_CID2080" V 1225 1700 35  0001 C CNN
F 2 "dtusat:C0402" V 1425 1700 35  0001 C CNN
F 3 "" H 1100 1700 30  0000 C CNN
F 4 "100nF" V 1168 1616 35  0000 L CNN "Component value"
F 5 "2080" V 1275 1700 35  0001 C CNN "CID"
F 6 "X7R" V 1325 1700 35  0001 C CNN "Dielectric"
F 7 "10%" V 1375 1700 35  0001 C CNN "Tolerance"
	1    1100 1700
	1    0    0    -1  
$EndComp
$Comp
L dtusat-capacitors:C_0402_100nF_CID2080 C?
U 1 1 5DEFEE38
P 1300 1700
F 0 "C?" V 1232 1664 35  0000 L CNN
F 1 "C_0402_100nF_CID2080" V 1425 1700 35  0001 C CNN
F 2 "dtusat:C0402" V 1625 1700 35  0001 C CNN
F 3 "" H 1300 1700 30  0000 C CNN
F 4 "100nF" V 1366 1616 35  0000 L CNN "Component value"
F 5 "2080" V 1475 1700 35  0001 C CNN "CID"
F 6 "X7R" V 1525 1700 35  0001 C CNN "Dielectric"
F 7 "10%" V 1575 1700 35  0001 C CNN "Tolerance"
	1    1300 1700
	1    0    0    -1  
$EndComp
$Comp
L dtusat-capacitors:C_0402_100nF_CID2080 C?
U 1 1 5DEFF254
P 1500 1700
F 0 "C?" V 1432 1664 35  0000 L CNN
F 1 "C_0402_100nF_CID2080" V 1625 1700 35  0001 C CNN
F 2 "dtusat:C0402" V 1825 1700 35  0001 C CNN
F 3 "" H 1500 1700 30  0000 C CNN
F 4 "100nF" V 1566 1616 35  0000 L CNN "Component value"
F 5 "2080" V 1675 1700 35  0001 C CNN "CID"
F 6 "X7R" V 1725 1700 35  0001 C CNN "Dielectric"
F 7 "10%" V 1775 1700 35  0001 C CNN "Tolerance"
	1    1500 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 1600 1500 1500
Wire Wire Line
	1500 1500 1575 1500
Wire Wire Line
	1575 1425 1300 1425
Wire Wire Line
	1300 1425 1300 1600
Wire Wire Line
	1575 1350 1100 1350
Wire Wire Line
	1100 1350 1100 1600
Wire Wire Line
	1575 1275 900  1275
Wire Wire Line
	900  1275 900  1600
Wire Wire Line
	900  1800 1100 1800
Connection ~ 1100 1800
Wire Wire Line
	1100 1800 1200 1800
Connection ~ 1300 1800
Wire Wire Line
	1300 1800 1500 1800
$Comp
L dtusat-power:GND #PWR?
U 1 1 5DF11F22
P 1200 1800
F 0 "#PWR?" H 1200 1550 35  0001 C CNN
F 1 "GND" H 1200 1638 35  0000 C CNN
F 2 "" H 1200 1800 35  0000 C CNN
F 3 "" H 1200 1800 35  0000 C CNN
	1    1200 1800
	1    0    0    -1  
$EndComp
Connection ~ 1200 1800
Wire Wire Line
	1200 1800 1300 1800
$Comp
L dtusat-power:GND #PWR?
U 1 1 5DF1275E
P 1500 2275
F 0 "#PWR?" H 1500 2025 35  0001 C CNN
F 1 "GND" H 1500 2113 35  0000 C CNN
F 2 "" H 1500 2275 35  0000 C CNN
F 3 "" H 1500 2275 35  0000 C CNN
	1    1500 2275
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 2275 1575 2275
$Comp
L dtusat-capacitors:C_0402_10nF_CID2082 C?
U 1 1 5DF1314F
P 1450 3175
F 0 "C?" V 1381 3139 35  0000 L CNN
F 1 "C_0402_10nF_CID2082" V 1575 3175 35  0001 C CNN
F 2 "dtusat:C0402" V 1775 3175 35  0001 C CNN
F 3 "" H 1450 3175 30  0000 C CNN
F 4 "10nF" V 1518 3108 35  0000 L CNN "Component value"
F 5 "2082" V 1625 3175 35  0001 C CNN "CID"
F 6 "X7R" V 1675 3175 35  0001 C CNN "Dielectric"
F 7 "10%" V 1725 3175 35  0001 C CNN "Tolerance"
	1    1450 3175
	1    0    0    -1  
$EndComp
$Comp
L dtusat-capacitors:C_0402_47pF_CID2081 C?
U 1 1 5DF13D1B
P 1225 3175
F 0 "C?" V 1157 3139 35  0000 L CNN
F 1 "C_0402_47pF_CID2081" V 1350 3175 35  0001 C CNN
F 2 "dtusat:C0402" V 1550 3175 35  0001 C CNN
F 3 "" H 1225 3175 30  0000 C CNN
F 4 "47pF" V 1296 3108 35  0000 L CNN "Component value"
F 5 "2081" V 1400 3175 35  0001 C CNN "CID"
F 6 "C0G" V 1450 3175 35  0001 C CNN "Dielectric"
F 7 "5%" V 1500 3175 35  0001 C CNN "Tolerance"
	1    1225 3175
	1    0    0    -1  
$EndComp
Wire Wire Line
	1225 3075 1450 3075
Connection ~ 1450 3075
Wire Wire Line
	1450 3075 1575 3075
$Comp
L dtusat-power:GND #PWR?
U 1 1 5DF18373
P 1225 3275
F 0 "#PWR?" H 1225 3025 35  0001 C CNN
F 1 "GND" H 1225 3113 35  0000 C CNN
F 2 "" H 1225 3275 35  0000 C CNN
F 3 "" H 1225 3275 35  0000 C CNN
	1    1225 3275
	1    0    0    -1  
$EndComp
$Comp
L dtusat-power:GND #PWR?
U 1 1 5DF18961
P 1450 3275
F 0 "#PWR?" H 1450 3025 35  0001 C CNN
F 1 "GND" H 1450 3113 35  0000 C CNN
F 2 "" H 1450 3275 35  0000 C CNN
F 3 "" H 1450 3275 35  0000 C CNN
	1    1450 3275
	1    0    0    -1  
$EndComp
NoConn ~ 2600 2175
$Comp
L dtusat-inductors:L_0603_33nH_CID3014 L?
U 1 1 5DF24B92
P 2075 3250
F 0 "L?" V 2032 3249 35  0000 C CNN
F 1 "L_0603_33nH_CID3014" V 2175 3250 35  0001 C CNN
F 2 "dtusat:L0603" V 2375 3250 35  0001 C CNN
F 3 "" H 2075 3250 30  0000 C CNN
F 4 "33nH" V 2107 3250 35  0000 C CNN "Component value"
F 5 "3014" V 2225 3250 35  0001 C CNN "CID"
F 6 "SMD" V 2275 3250 35  0001 C CNN "Technology"
F 7 "5%" V 2325 3250 35  0001 C CNN "Tolerance"
	1    2075 3250
	0    1    1    0   
$EndComp
$Comp
L dtusat-inductors:L_0402_2.7nH_CID3035 L?
U 1 1 5DF26C3D
P 2700 3075
F 0 "L?" V 2660 3075 35  0000 C CNN
F 1 "L_0402_2.7nH_CID3035" V 2800 3075 35  0001 C CNN
F 2 "dtusat:L0402" V 3000 3075 35  0001 C CNN
F 3 "" H 2700 3075 30  0000 C CNN
F 4 "2.7nH" V 2728 3075 35  0000 C CNN "Component value"
F 5 "3035" V 2850 3075 35  0001 C CNN "CID"
F 6 "SMD" V 2900 3075 35  0001 C CNN "Technology"
F 7 "0.1nH" V 2950 3075 35  0001 C CNN "Tolerance"
	1    2700 3075
	0    1    1    0   
$EndComp
Connection ~ 1575 2275
Wire Wire Line
	1575 2200 1575 2275
Connection ~ 1575 2200
Connection ~ 1575 2125
Wire Wire Line
	1575 2125 1575 2200
Wire Wire Line
	1575 2050 1575 2125
$Comp
L dtusat-ic:X1E0000210162_CID142 X?
U 1 1 5DF18FBD
P 3250 2750
F 0 "X?" V 3178 2715 35  0000 L CNN
F 1 "X1E0000210162_CID142" V 3500 2750 35  0001 C CNN
F 2 "" V 3180 2750 30  0000 C CNN
F 3 "http://www.farnell.com/datasheets/526621.pdf?_ga=1.197279019.583306404.1491382241" V 3250 2750 35  0001 C CNN
F 4 "142" V 3550 2750 35  0001 C CNN "CID"
	1    3250 2750
	1    0    0    -1  
$EndComp
$Comp
L dtusat-capacitors:C_0402_15pF_CID2083 C?
U 1 1 5DF195DB
P 3425 2650
F 0 "C?" V 3355 2614 35  0000 L CNN
F 1 "C_0402_15pF_CID2083" V 3550 2650 35  0001 C CNN
F 2 "dtusat:C0402" V 3750 2650 35  0001 C CNN
F 3 "" H 3425 2650 30  0000 C CNN
F 4 "15pF" V 3494 2583 35  0000 L CNN "Component value"
F 5 "2083" V 3600 2650 35  0001 C CNN "CID"
F 6 "C0G" V 3650 2650 35  0001 C CNN "Dielectric"
F 7 "1%" V 3700 2650 35  0001 C CNN "Tolerance"
	1    3425 2650
	0    1    1    0   
$EndComp
$Comp
L dtusat-capacitors:C_0402_15pF_CID2083 C?
U 1 1 5DF1A15F
P 3425 2850
F 0 "C?" V 3356 2814 35  0000 L CNN
F 1 "C_0402_15pF_CID2083" V 3550 2850 35  0001 C CNN
F 2 "dtusat:C0402" V 3750 2850 35  0001 C CNN
F 3 "" H 3425 2850 30  0000 C CNN
F 4 "15pF" V 3493 2782 35  0000 L CNN "Component value"
F 5 "2083" V 3600 2850 35  0001 C CNN "CID"
F 6 "C0G" V 3650 2850 35  0001 C CNN "Dielectric"
F 7 "1%" V 3700 2850 35  0001 C CNN "Tolerance"
	1    3425 2850
	0    1    1    0   
$EndComp
$Comp
L dtusat-power:GND #PWR?
U 1 1 5DF23186
P 3575 2875
F 0 "#PWR?" H 3575 2625 35  0001 C CNN
F 1 "GND" H 3575 2713 35  0000 C CNN
F 2 "" H 3575 2875 35  0000 C CNN
F 3 "" H 3575 2875 35  0000 C CNN
	1    3575 2875
	1    0    0    -1  
$EndComp
Connection ~ 3250 2650
Wire Wire Line
	3250 2650 3325 2650
Wire Wire Line
	3525 2650 3525 2850
Wire Wire Line
	3575 2875 3575 2850
Wire Wire Line
	3575 2850 3525 2850
Connection ~ 3525 2850
Wire Wire Line
	3325 2850 3250 2850
Connection ~ 3250 2850
NoConn ~ 2600 2925
Wire Wire Line
	2600 2850 2600 2775
Wire Wire Line
	2600 2700 2600 2650
$Comp
L dtusat-ic:SX1272IMLTRICT-ND_CID104 U2
U 1 1 5DEF895B
P 2100 2225
F 0 "U2" H 2087 3357 35  0000 C CNN
F 1 "SX1272IMLTRICT-ND_CID104" H 2100 1275 35  0001 C CNN
F 2 "dtusat:QFN-28" H 2087 3288 35  0000 C CNN
F 3 "http://www.semtech.com/images/datasheet/sx1272.pdf" H 2100 1225 35  0001 C CNN
F 4 "104" H 2100 1125 35  0001 C CNN "CID"
	1    2100 2225
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 2650 3250 2650
Wire Wire Line
	2600 2850 3250 2850
Wire Wire Line
	1575 3250 1575 3075
Wire Wire Line
	1575 3250 1975 3250
Connection ~ 1575 3075
Text Label 3250 3000 2    35   ~ 0
SX1272_RF_RX
Wire Wire Line
	2600 3000 3250 3000
Wire Wire Line
	2175 3250 2800 3250
Wire Wire Line
	2800 3075 2800 3250
Connection ~ 2800 3250
Wire Wire Line
	2800 3250 3250 3250
Text Label 3250 3250 2    35   ~ 0
SX1272_RF_TX
Wire Wire Line
	2600 2550 3050 2550
Wire Wire Line
	2600 2475 3050 2475
Wire Wire Line
	2600 2400 3050 2400
Wire Wire Line
	2600 2325 3050 2325
Text Label 3050 2550 2    35   ~ 0
RF_CS
Text Label 3050 2475 2    35   ~ 0
MOSI
Text Label 3050 2400 2    35   ~ 0
MISO
Text Label 3050 2325 2    35   ~ 0
SCK
$Comp
L dtusat-power:GND #PWR?
U 1 1 5DF42A4C
P 2725 2025
F 0 "#PWR?" H 2725 1775 35  0001 C CNN
F 1 "GND" H 2725 1863 35  0000 C CNN
F 2 "" H 2725 2025 35  0000 C CNN
F 3 "" H 2725 2025 35  0000 C CNN
	1    2725 2025
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 2025 2725 2025
Wire Wire Line
	2600 2100 2600 2025
Connection ~ 2600 2025
Text Label 2950 1800 2    35   ~ 0
RXTX_SW
$Comp
L dtusat-resistors:R_0402_10k_CID1101 R?
U 1 1 5DF46196
P 3050 1875
F 0 "R?" V 3000 1875 35  0000 C CNN
F 1 "R_0402_10k_CID1101" V 3150 1875 35  0001 C CNN
F 2 "dtusat:R0402" V 3350 1875 35  0001 C CNN
F 3 "" H 3050 1875 30  0000 C CNN
F 4 "10k" V 3100 1875 35  0000 C CNN "Component value"
F 5 "1101" V 3200 1875 35  0001 C CNN "CID"
F 6 "Thickfilm" V 3250 1875 35  0001 C CNN "Technology"
F 7 "1%" V 3300 1875 35  0001 C CNN "Tolerance"
	1    3050 1875
	0    1    1    0   
$EndComp
Text Label 2950 1875 2    35   ~ 0
Reset_SX
Wire Wire Line
	2600 1800 2950 1800
Wire Wire Line
	2600 1875 2950 1875
$Comp
L dtusat-power:GND #PWR?
U 1 1 5DF4A280
P 3200 1875
F 0 "#PWR?" H 3200 1625 35  0001 C CNN
F 1 "GND" H 3200 1713 35  0000 C CNN
F 2 "" H 3200 1875 35  0000 C CNN
F 3 "" H 3200 1875 35  0000 C CNN
	1    3200 1875
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1875 3150 1875
Wire Wire Line
	2600 1650 2950 1650
Wire Wire Line
	2600 1575 2950 1575
Wire Wire Line
	2600 1500 2950 1500
Wire Wire Line
	2600 1425 2950 1425
Wire Wire Line
	2600 1350 2950 1350
Wire Wire Line
	2600 1275 2950 1275
Text Label 2950 1650 2    35   ~ 0
DIO5
Text Label 2950 1575 2    35   ~ 0
DIO4
Text Label 2950 1500 2    35   ~ 0
DIO3
Text Label 2950 1425 2    35   ~ 0
DIO2
Text Label 2950 1350 2    35   ~ 0
DIO1
Text Label 2950 1275 2    35   ~ 0
DIO0
$EndSCHEMATC
