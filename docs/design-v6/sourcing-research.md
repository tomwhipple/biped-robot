# v6 Biped Sourcing Research

## 1. Feetech STS3250
- **Official Datasheet/2D Drawing**: Found via AIFITLAB and official Feetech documentation.
- **Dimensions**: 45.22 x 24.72 x 35 mm.
- **Weight**: 74.5 ± 1 g.
- **Case/Horn Compatibility**: 
    - **Horn**: 25T / OD 5.9 mm. Identical to STS3215.
    - **Case/Envelope**: Identified as "almost the same physical envelope" as the STS3215, but utilizes an Aluminum case (STS3215 uses PA+GF plastic).
- **Performance**: 
    - **Stall Torque**: 50 kg·cm @ 12V.
    - **No-load Speed**: 0.133 sec/60° @ 12V (approx 75 RPM).
- **Pricing (US)**:
    - OpenELAB: $73.99.
    - AIFITLAB: $88.00 - $97.00.
- **Lead Time**: 2-7 business days (US Warehouse).
- **Stiffness/Deflection**: 
    - Published backlash: ≤ 0.5°.
    - Measured backlash (Robonine): Single STS3250 unloaded = 0.13°, loaded = 0.33° (best in class compared to STS3215/HLS3950M).
    - No explicit "stiffness" (N/mm) published, but measured as the most stable in the 50kg class.

## 2. 3S Protection Board (Smart BMS)
- **Candidate 1: JBD (Jiabaida) SP04S010**
    - **Current**: 20A - 35A continuous.
    - **Telemetry**: UART / RS485 / Bluetooth.
    - **Dimensions**: 80 x 60 x 12 mm (20A) or 80 x 60 x 20 mm (30A/35A).
    - **Price**: ~$43 - $50.
- **Candidate 2: Daly Smart BMS (R05J)**
    - **Current**: 30A continuous / 15A charge.
    - **Telemetry**: UART / Bluetooth.
    - **Dimensions**: 100 x 65 x 13 mm.
    - **Price**: Variable; typically $50 - $100 for smart variants.

## 3. Raspberry Pi 4B & Camera Module 3 Wide
- **Pi 4B Mechanicals**:
    - **Board Size**: 85 x 56 mm.
    - **Mounting Holes**: 58 x 49 mm rectangle (center-to-center), 2.7 mm diameter (for M2.5).
    - **Stack Height**: USB/Ethernet stack is the tallest point.
- **Camera Module 3 Wide**:
    - **Board Size**: 25 x 24 mm.
    - **Mounting Holes**: 21 x 12.5 mm (identical to Camera Module 2).
    - **Dimensions**: 25 x 24 x 12.4 mm (taller than standard v3 due to wide lens).
    - **Lens Position**: Centered on the sensor; protrudes more than v2.
    - **Ribbon Length**: 200 mm.

## 4. Pololu D24V50F5
- **Dimensions**: 17.8 x 20.3 x 8.8 mm (0.7" x 0.8" x 0.35").
- **Mounting Holes**: Two 0.086" (2.18 mm) holes for #2 or M2 screws.
- **Hole Spacing**: 0.53" (13.5 mm) horizontal and 0.63" (16.0 mm) vertical.
- **Documentation**: Official dimensions PDF verified on Pololu site.

## 5. 3S LiPo Packs (11.1V, 2200-2600mAh)
- **Constraint**: <= 105 x 36 x 26 mm.
- **Candidate 1: Yowoo 3S 2200mAh**
    - **Dimensions**: 105 x 36 x 26 mm (Exact match).
    - **Connector**: XT60.
    - **Weight**: 208 g.
- **Candidate 2: Admiral 2200mAh (EPR22003X6)**
    - **Dimensions**: 105 x 34 x 22 mm (Within limits).
    - **Connector**: XT60.
- **Candidate 3: Yowoo 3S 2600mAh**
    - **Dimensions**: 120 x 36 x 25 mm (**Exceeds length limit**).
- **Conclusion**: 2200mAh is the safe bet for the 105mm length constraint. 2600mAh packs typically exceed 115mm+.
