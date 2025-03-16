SamacSys ECAD Model
971781/1070213/2.50/32/3/Integrated Circuit

DESIGNSPARK_INTERMEDIATE_ASCII

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r145_30"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.3) (shapeHeight 1.45))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Default"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 50 mils)
			(strokeWidth 5 mils)
		)
	)
	(patternDef "QFP50P700X700X120-32N" (originalName "QFP50P700X700X120-32N")
		(multiLayer
			(pad (padNum 1) (padStyleRef r145_30) (pt -3.225, 1.75) (rotation 90))
			(pad (padNum 2) (padStyleRef r145_30) (pt -3.225, 1.25) (rotation 90))
			(pad (padNum 3) (padStyleRef r145_30) (pt -3.225, 0.75) (rotation 90))
			(pad (padNum 4) (padStyleRef r145_30) (pt -3.225, 0.25) (rotation 90))
			(pad (padNum 5) (padStyleRef r145_30) (pt -3.225, -0.25) (rotation 90))
			(pad (padNum 6) (padStyleRef r145_30) (pt -3.225, -0.75) (rotation 90))
			(pad (padNum 7) (padStyleRef r145_30) (pt -3.225, -1.25) (rotation 90))
			(pad (padNum 8) (padStyleRef r145_30) (pt -3.225, -1.75) (rotation 90))
			(pad (padNum 9) (padStyleRef r145_30) (pt -1.75, -3.225) (rotation 0))
			(pad (padNum 10) (padStyleRef r145_30) (pt -1.25, -3.225) (rotation 0))
			(pad (padNum 11) (padStyleRef r145_30) (pt -0.75, -3.225) (rotation 0))
			(pad (padNum 12) (padStyleRef r145_30) (pt -0.25, -3.225) (rotation 0))
			(pad (padNum 13) (padStyleRef r145_30) (pt 0.25, -3.225) (rotation 0))
			(pad (padNum 14) (padStyleRef r145_30) (pt 0.75, -3.225) (rotation 0))
			(pad (padNum 15) (padStyleRef r145_30) (pt 1.25, -3.225) (rotation 0))
			(pad (padNum 16) (padStyleRef r145_30) (pt 1.75, -3.225) (rotation 0))
			(pad (padNum 17) (padStyleRef r145_30) (pt 3.225, -1.75) (rotation 90))
			(pad (padNum 18) (padStyleRef r145_30) (pt 3.225, -1.25) (rotation 90))
			(pad (padNum 19) (padStyleRef r145_30) (pt 3.225, -0.75) (rotation 90))
			(pad (padNum 20) (padStyleRef r145_30) (pt 3.225, -0.25) (rotation 90))
			(pad (padNum 21) (padStyleRef r145_30) (pt 3.225, 0.25) (rotation 90))
			(pad (padNum 22) (padStyleRef r145_30) (pt 3.225, 0.75) (rotation 90))
			(pad (padNum 23) (padStyleRef r145_30) (pt 3.225, 1.25) (rotation 90))
			(pad (padNum 24) (padStyleRef r145_30) (pt 3.225, 1.75) (rotation 90))
			(pad (padNum 25) (padStyleRef r145_30) (pt 1.75, 3.225) (rotation 0))
			(pad (padNum 26) (padStyleRef r145_30) (pt 1.25, 3.225) (rotation 0))
			(pad (padNum 27) (padStyleRef r145_30) (pt 0.75, 3.225) (rotation 0))
			(pad (padNum 28) (padStyleRef r145_30) (pt 0.25, 3.225) (rotation 0))
			(pad (padNum 29) (padStyleRef r145_30) (pt -0.25, 3.225) (rotation 0))
			(pad (padNum 30) (padStyleRef r145_30) (pt -0.75, 3.225) (rotation 0))
			(pad (padNum 31) (padStyleRef r145_30) (pt -1.25, 3.225) (rotation 0))
			(pad (padNum 32) (padStyleRef r145_30) (pt -1.75, 3.225) (rotation 0))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Default") (isVisible True))
		)
		(layerContents (layerNumRef 30)
			(line (pt -4.2 4.2) (pt 4.2 4.2) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt 4.2 4.2) (pt 4.2 -4.2) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt 4.2 -4.2) (pt -4.2 -4.2) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt -4.2 -4.2) (pt -4.2 4.2) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.5 2.5) (pt 2.5 2.5) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 2.5 2.5) (pt 2.5 -2.5) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 2.5 -2.5) (pt -2.5 -2.5) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.5 -2.5) (pt -2.5 2.5) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.5 2) (pt -2 2.5) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.15 2.15) (pt 2.15 2.15) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 2.15 2.15) (pt 2.15 -2.15) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 2.15 -2.15) (pt -2.15 -2.15) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.15 -2.15) (pt -2.15 2.15) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -3.7, 2.5) (radius 0.125) (startAngle 0.0) (sweepAngle 0.0) (width 0.25))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -3.7, 2.5) (radius 0.125) (startAngle 180.0) (sweepAngle 180.0) (width 0.25))
		)
	)
	(symbolDef "ADS114S08BIPBS" (originalName "ADS114S08BIPBS")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 4) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 5) (pt 0 mils -400 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -425 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 6) (pt 0 mils -500 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -525 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 7) (pt 0 mils -600 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -625 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 8) (pt 0 mils -700 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -725 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 9) (pt 500 mils -1600 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 525 mils -1370 mils) (rotation 90]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 10) (pt 600 mils -1600 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 625 mils -1370 mils) (rotation 90]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 11) (pt 700 mils -1600 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 725 mils -1370 mils) (rotation 90]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 12) (pt 800 mils -1600 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 825 mils -1370 mils) (rotation 90]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 13) (pt 900 mils -1600 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 925 mils -1370 mils) (rotation 90]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 14) (pt 1000 mils -1600 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1025 mils -1370 mils) (rotation 90]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 15) (pt 1100 mils -1600 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1125 mils -1370 mils) (rotation 90]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 16) (pt 1200 mils -1600 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1225 mils -1370 mils) (rotation 90]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 17) (pt 1800 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1570 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 18) (pt 1800 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1570 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 19) (pt 1800 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1570 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 20) (pt 1800 mils -300 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1570 mils -325 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 21) (pt 1800 mils -400 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1570 mils -425 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 22) (pt 1800 mils -500 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1570 mils -525 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 23) (pt 1800 mils -600 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1570 mils -625 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 24) (pt 1800 mils -700 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1570 mils -725 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 25) (pt 500 mils 900 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 525 mils 670 mils) (rotation 90]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 26) (pt 600 mils 900 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 625 mils 670 mils) (rotation 90]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 27) (pt 700 mils 900 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 725 mils 670 mils) (rotation 90]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 28) (pt 800 mils 900 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 825 mils 670 mils) (rotation 90]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 29) (pt 900 mils 900 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 925 mils 670 mils) (rotation 90]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 30) (pt 1000 mils 900 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1025 mils 670 mils) (rotation 90]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 31) (pt 1100 mils 900 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1125 mils 670 mils) (rotation 90]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 32) (pt 1200 mils 900 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1225 mils 670 mils) (rotation 90]) (justify "Right") (textStyleRef "Default"))
		))
		(line (pt 200 mils 700 mils) (pt 1600 mils 700 mils) (width 6 mils))
		(line (pt 1600 mils 700 mils) (pt 1600 mils -1400 mils) (width 6 mils))
		(line (pt 1600 mils -1400 mils) (pt 200 mils -1400 mils) (width 6 mils))
		(line (pt 200 mils -1400 mils) (pt 200 mils 700 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 1650 mils 900 mils) (justify Left) (isVisible True) (textStyleRef "Default"))

	)
	(compDef "ADS114S08BIPBS" (originalName "ADS114S08BIPBS") (compHeader (numPins 32) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "AINCOM") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "AIN5") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "3" (pinName "AIN4") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "4" (pinName "AIN3") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "5" (pinName "AIN2") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "6" (pinName "AIN1") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "7" (pinName "AIN0") (partNum 1) (symPinNum 7) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "8" (pinName "START/SYNC") (partNum 1) (symPinNum 8) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "9" (pinName "__CS") (partNum 1) (symPinNum 9) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "10" (pinName "DIN") (partNum 1) (symPinNum 10) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "11" (pinName "SCLK") (partNum 1) (symPinNum 11) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "12" (pinName "DOUT/__DRDY") (partNum 1) (symPinNum 12) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "13" (pinName "__DRDY") (partNum 1) (symPinNum 13) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "14" (pinName "DGND") (partNum 1) (symPinNum 14) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "15" (pinName "IOVDD") (partNum 1) (symPinNum 15) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "16" (pinName "DVDD") (partNum 1) (symPinNum 16) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "24" (pinName "REFCOM") (partNum 1) (symPinNum 17) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "23" (pinName "REFOUT") (partNum 1) (symPinNum 18) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "22" (pinName "GPIO0/AIN8") (partNum 1) (symPinNum 19) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "21" (pinName "GPIO1/AIN9") (partNum 1) (symPinNum 20) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "20" (pinName "GPIO2/AIN10") (partNum 1) (symPinNum 21) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "19" (pinName "GPIO3/AIN11") (partNum 1) (symPinNum 22) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "18" (pinName "__RESET") (partNum 1) (symPinNum 23) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "17" (pinName "CLK") (partNum 1) (symPinNum 24) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "32" (pinName "REFP1/AIN6") (partNum 1) (symPinNum 25) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "31" (pinName "REFN1/AIN7") (partNum 1) (symPinNum 26) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "30" (pinName "REFP0") (partNum 1) (symPinNum 27) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "29" (pinName "REFN0") (partNum 1) (symPinNum 28) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "28" (pinName "AVSS_2") (partNum 1) (symPinNum 29) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "27" (pinName "AVSS_1") (partNum 1) (symPinNum 30) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "26" (pinName "AVDD") (partNum 1) (symPinNum 31) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "25" (pinName "NC") (partNum 1) (symPinNum 32) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "ADS114S08BIPBS"))
		(attachedPattern (patternNum 1) (patternName "QFP50P700X700X120-32N")
			(numPads 32)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
				(padNum 6) (compPinRef "6")
				(padNum 7) (compPinRef "7")
				(padNum 8) (compPinRef "8")
				(padNum 9) (compPinRef "9")
				(padNum 10) (compPinRef "10")
				(padNum 11) (compPinRef "11")
				(padNum 12) (compPinRef "12")
				(padNum 13) (compPinRef "13")
				(padNum 14) (compPinRef "14")
				(padNum 15) (compPinRef "15")
				(padNum 16) (compPinRef "16")
				(padNum 17) (compPinRef "17")
				(padNum 18) (compPinRef "18")
				(padNum 19) (compPinRef "19")
				(padNum 20) (compPinRef "20")
				(padNum 21) (compPinRef "21")
				(padNum 22) (compPinRef "22")
				(padNum 23) (compPinRef "23")
				(padNum 24) (compPinRef "24")
				(padNum 25) (compPinRef "25")
				(padNum 26) (compPinRef "26")
				(padNum 27) (compPinRef "27")
				(padNum 28) (compPinRef "28")
				(padNum 29) (compPinRef "29")
				(padNum 30) (compPinRef "30")
				(padNum 31) (compPinRef "31")
				(padNum 32) (compPinRef "32")
			)
		)
		(attr "Mouser Part Number" "595-ADS114S08BIPBS")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Texas-Instruments/ADS114S08BIPBS?qs=BZBei1rCqCDQBH%2FvFX3yrQ%3D%3D")
		(attr "Manufacturer_Name" "Texas Instruments")
		(attr "Manufacturer_Part_Number" "ADS114S08BIPBS")
		(attr "Description" "16-Bit, 4kSPS, 12-Ch Delta-Sigma ADC With PGA and Voltage Reference for Low-Cost Applications")
		(attr "Datasheet Link" "http://www.ti.com/lit/ds/symlink/ads114s06b.pdf")
		(attr "Height" "1.2 mm")
	)

)
