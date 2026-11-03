"""Power-lite board: every part, its symbol, footprint, MPN, datasheet and the net of every pin (the spec).

The design and its decisions: docs/design-v6/2026-11-02-power-lite-options.md. Pin orders of the
connectors that carry over from rev B follow pcb/power-board/harness.md.
"""
PROJECT = "power-lite"

DS = {
    "Amass XT60PW-M": "https://www.amass.com.cn/pdf/XT60PW.pdf",
    "JST B4B-XH-A": "https://www.jst-mfg.com/product/pdf/eng/eXH.pdf",
    "JST B2B-XH-A": "https://www.jst-mfg.com/product/pdf/eng/eXH.pdf",
    "JST B2B-PH-K-S": "https://www.jst-mfg.com/product/pdf/eng/ePH.pdf",
    "JST B8B-PH-K-S": "https://www.jst-mfg.com/product/pdf/eng/ePH.pdf",
    "Molex 22-03-5035": "https://www.molex.com/en-us/products/part-detail/22035035",
    "Keystone 3568": "https://www.keyelco.com/userAssets/file/M65p73.pdf",
    "TE 1-1414610-0": "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=V23134X0000A002"
                      "&DocType=Data%20Sheet&DocLang=English&DocFormat=pdf",
    "Good-Ark 1N4007": "https://goodarksemi.com/docs/datasheets/general_purpose_rectifiers/1N400x.pdf",
    "Littelfuse 1.5KE16A": "https://www.littelfuse.com/media?resourcetype=datasheets&itemid=9D02C7D2-86A5-4BFF-9251-"
                           "966D6AF02BF7&filename=Littelfuse-TVS-Diode-1-5KE-Datasheet",
    "Panasonic EEU-FR1E102": "https://industrial.panasonic.com/cdbs/www-data/pdf/RDF0000/ABA0000C1215.pdf",
    "onsemi PN2222A": "https://www.onsemi.com/pdf/datasheet/pn2222a-d.pdf",
    "Kingbright WP710A10GD": "https://www.kingbrightusa.com/images/catalog/SPEC/WP710A10GD.pdf",
    "Kingbright WP710A10ID": "https://www.kingbrightusa.com/images/catalog/SPEC/WP710A10ID.pdf",
}
DS_MFR = "https://www.yageo.com/upload/media/product/productsearch/datasheet/lr/Yageo_LR_MFR_1.pdf"

parts = {}


def part(ref, lib_id, value, fp, nets, mpn="", desc=""):
    ds = DS_MFR if mpn.startswith("Yageo MFR") else DS.get(mpn, "~")
    parts[ref] = dict(ref=ref, lib_id=lib_id, value=value, fp=fp, nets=nets, mpn=mpn, desc=desc, datasheet=ds)


def R(ref, value, a, b, mpn, desc=""):
    part(ref, "Device:R", value, "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal",
         {"1": a, "2": b}, mpn, desc)


def WIRE(ref, value, netname, size, desc):
    fp = {"heavy": "Connector_Wire:SolderWire-2sqmm_1x01_D2mm_OD3.9mm",
          "tap": "Connector_Wire:SolderWire-0.5sqmm_1x01_D0.9mm_OD2.1mm"}[size]
    part(ref, "Connector_Generic:Conn_01x01", value, fp, {"1": netname}, "", desc)


# ---- the pack and the BMS (ShillehTek CF-3SJH25A-C, switches the negative lead, P+ = B1+)
part("J1", "Connector_Generic:Conn_01x02", "XT60 pack", "power-lite:AMASS_XT60PW-M_1x02_P7.20mm_Horizontal_EdgeMount",
     {"1": "BATN", "2": "BATP"}, "Amass XT60PW-M",
     "pack in; pad 1 = angled-side contact (-), pad 2 = straight side (+), as rev B J1")
part("J2", "Connector_Generic:Conn_01x04", "Balance 3S", "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical",
     {"1": "BATN", "2": "CELL1", "3": "CELL2", "4": "BATP"}, "JST B4B-XH-A",
     "the pack's balance lead: 1 = B-, 4 = B+ (harness.md P2)")
WIRE("W1", "BMS B-", "BATN", "heavy", "to the BMS's B- pad")
WIRE("W2", "BMS P-", "GND", "heavy", "from the BMS's P- pad: the system ground")
WIRE("W3", "BMS B3", "CELL1", "tap", "to the BMS's B3 (3.7 V) pad")
WIRE("W4", "BMS B2", "CELL2", "tap", "to the BMS's B2 (7.4 V) pad")
WIRE("W5", "BMS B1+", "BATP", "tap", "to the BMS's B1+ (11.1 V) pad, its top-cell sense")

# ---- the main switch (Cole Hersee 57000-01, panel mount, wired)
WIRE("W6", "SW in", "BATP", "heavy", "to the main switch")
WIRE("W7", "SW out", "VIN", "heavy", "from the main switch")
part("D3", "Device:D_Zener", "1.5KE16A", "Diode_THT:D_DO-201_P15.24mm_Horizontal",
     {"1": "VIN", "2": "GND"}, "Littelfuse 1.5KE16A", "unidirectional TVS on the switched input")

# ---- fuses
part("F1", "Device:Fuse", "15A", "Fuse:Fuseholder_Blade_Mini_Keystone_3568", {"1": "VIN", "2": "SRV_F"},
     "Keystone 3568", "servo rail: holder for a Littelfuse 0297015.WXNV 15 A MINI blade fuse")
part("F2", "Device:Fuse", "3A", "Fuse:Fuseholder_Blade_Mini_Keystone_3568", {"1": "VIN", "2": "LOGIC"},
     "Keystone 3568", "logic and E-stop coil: holder for a Littelfuse 0297003.WXNV 3 A MINI blade fuse")

# ---- logic feed to the General Driver (H1), as rev B J3
part("J3", "Connector_Generic:Conn_01x02", "To General Driver H1", "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical",
     {"1": "LOGIC", "2": "GND"}, "JST B2B-XH-A", "pin 1 to pin 1 (harness.md P3)")

# ---- servo rail: relay, coil drive, E-stop
part("K1", "Relay:Relay_SPST-NO", "1-1414610-0", "power-lite:TE_PowerRelay_F7_PCB",
     {"A1": "COIL_HI", "A2": "COIL_LO", "13": "SRV_F", "14": "SERVOP"}, "TE 1-1414610-0",
     "TE Power Relay F7 V23134-J0052-X455: 1 form A, 70 A, 12 V coil 90 R, operate 7.2 V, 470 R across the coil")
part("D1", "Device:D", "1N4007", "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal",
     {"1": "COIL_HI", "2": "COIL_LO"}, "Good-Ark 1N4007", "coil flyback: the built-in 470 R alone gives ~63 V")
part("J11", "Connector_Generic:Conn_01x02", "E-STOP (NC)", "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical",
     {"1": "LOGIC", "2": "COIL_HI"}, "JST B2B-PH-K-S",
     "a latching switch's NC contact; opening it drops K1. PH, unlike J3's XH, so the leads cannot swap")
part("Q1", "Transistor_BJT:PN2222A", "PN2222A", "Package_TO_SOT_THT:TO-92_Inline",
     {"1": "GND", "2": "Q1_B", "3": "COIL_LO"}, "onsemi PN2222A", "coil switch, driven by the ESP32's SERVO_EN")
R("R1", "470", "SERVO_EN", "Q1_B", "Yageo MFR-25FBF52-470R", "base drive, ~5 mA from 3.3 V")
R("R2", "10k", "Q1_B", "GND", "Yageo MFR-25FBF52-10K", "holds the rail off until the ESP32 drives SERVO_EN")

# ---- servo rail: bulk, TVS, distribution (J4-J8 as rev B: 1 data, 2 V+, 3 GND)
part("C1", "Device:C_Polarized", "1000u 25V", "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm",
     {"1": "SERVOP", "2": "GND"}, "Panasonic EEU-FR1E102", "servo rail bulk")
part("D2", "Device:D_Zener", "1.5KE16A", "Diode_THT:D_DO-201_P15.24mm_Horizontal",
     {"1": "SERVOP", "2": "GND"}, "Littelfuse 1.5KE16A", "unidirectional TVS on the servo rail")
for ref, name in (("J4", "R leg"), ("J5", "R upper"), ("J6", "L leg"), ("J7", "L upper")):
    part(ref, "Connector_Generic:Conn_01x03", f"Servo {name}", "Connector_Molex:Molex_SPOX_5267-03A_1x03_P2.50mm_Vertical",
         {"1": "SERVO_DATA", "2": "SERVOP", "3": "GND"}, "Molex 22-03-5035", "D V G, as the General Driver's H5")
part("J8", "Connector_Generic:Conn_01x03", "Data from H5", "Connector_Molex:Molex_SPOX_5267-03A_1x03_P2.50mm_Vertical",
     {"1": "SERVO_DATA", "2": None, "3": "GND"}, "Molex 22-03-5035", "V+ pin unconnected on purpose")

# ---- telemetry to the ESP32 (J12 keeps rev B's pin order; KILL, PWR_INT_N, IMON, FLT are gone)
R("R3", "47k", "SERVOP", "RAIL_V", "Yageo MFR-25FBF52-47K", "RAIL_V: 2.21 V at 12.6 V")
R("R4", "10k", "RAIL_V", "GND", "Yageo MFR-25FBF52-10K")
R("R5", "47k", "COIL_HI", "ESTOP_N", "Yageo MFR-25FBF52-47K", "ESTOP_N high = E-stop released (2.21 V at 12.6 V)")
R("R6", "10k", "ESTOP_N", "GND", "Yageo MFR-25FBF52-10K")
part("J12", "Connector_Generic:Conn_01x08", "Control (ESP32)", "Connector_JST:JST_PH_B8B-PH-K_1x08_P2.00mm_Vertical",
     {"1": None, "2": "SERVO_EN", "3": "GND", "4": None, "5": "RAIL_V", "6": "ESTOP_N", "7": None, "8": None},
     "JST B8B-PH-K-S", "rev B's order: 2 SERVO_EN (IO16), 3 GND, 5 RAIL_V (GPIO34), 6 ESTOP_N (IO4)")

# ---- indicators
R("R7", "4.7k", "LOGIC", "LED1_A", "Yageo MFR-25FBF52-4K7")
part("LED1", "Device:LED", "green", "LED_THT:LED_D3.0mm", {"1": "GND", "2": "LED1_A"}, "Kingbright WP710A10GD", "logic feed on")
R("R8", "4.7k", "SERVOP", "LED2_A", "Yageo MFR-25FBF52-4K7")
part("LED2", "Device:LED", "red", "LED_THT:LED_D3.0mm", {"1": "GND", "2": "LED2_A"}, "Kingbright WP710A10ID", "servo rail on")
