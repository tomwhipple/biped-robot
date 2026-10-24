# Power board: manufacturer documents

The manufacturer documents the [power board](../../../pcb/power-board/README.md)
is drawn from and built with, mirrored verbatim. The TI documents were retrieved
2026-10-06, the rest 2026-10-07. Not yet mirrored (the manufacturers' sites
refuse automated downloads): Littelfuse 0297015.WXNV and 0297003.WXNV (MINI 297 series), Molex 22-03-5035,
Panasonic EEU-FR1E102; the schematic's Datasheet field links their pages.

| file | what it is | source |
|---|---|---|
| [bq76922.pdf](bq76922.pdf) | BQ76922 3S–5S battery monitor and protector, datasheet SLUSE86A (rev. April 2024). Pin table 5-1, application section 8.2, unused pins table 8-3 | [ti.com](https://www.ti.com/lit/ds/symlink/bq76922.pdf) |
| [bq76922-trm-sluucg7.pdf](bq76922-trm-sluucg7.pdf) | BQ76922 technical reference manual SLUUCG7: registers, data memory, OTP programming | [ti.com](https://www.ti.com/lit/ug/sluucg7/sluucg7.pdf) |
| [bq76922evm-slvu957a.pdf](bq76922evm-slvu957a.pdf) | BQ76922EVM user's guide SLVU957A: the EVM schematic (figures 6-9 and 6-10) and BOM that the FET section copies | [ti.com](https://www.ti.com/lit/ug/slvu957a/slvu957a.pdf) |
| [tps4811-q1.pdf](tps4811-q1.pdf) | TPS4811-Q1 (TPS48111-Q1) high-side switch controller, datasheet SLUSEE5E (rev. April 2026). Design equations §8.3 and §9.2; DGX0019A land pattern | [ti.com](https://www.ti.com/lit/ds/symlink/tps4811-q1.pdf) |
| [csd17556q5b.pdf](csd17556q5b.pdf) | CSD17556Q5B 30 V N-channel NexFET, SON 5 × 6 mm | [ti.com](https://www.ti.com/lit/ds/symlink/csd17556q5b.pdf) |
| [css2h-2512.pdf](css2h-2512.pdf) | Bourns CSS2H-2512 series metal-element current-sense resistors. R1 = CSS2H-2512R-1L00FE (1 m, 5 W), R24 = CSS2H-2512K-2L00F (2 m, 5 W); ordering code: F = 1 %, suffix E = 7 inch mini reel (blank = 13 inch reel), R = Cu-Mn, K = Fe-Cr element | [bourns.com](https://www.bourns.com/docs/product-datasheets/css2h-2512.pdf) |
| [yageo-rc-l.pdf](yageo-rc-l.pdf) | Yageo RC_L series general-purpose thick-film chip resistors (RC0603FR-07xxxL, 1 %, 0603), V.14 November 2025 | [yageogroup.com](https://yageogroup.com/content/datasheet/asset/file/PYU-RC_GROUP_51_ROHS_L) |
| [vishay-crcw-e3.pdf](vishay-crcw-e3.pdf) | Vishay Dale CRCW e3 thick-film chip resistors (CRCW06035K10FKEA, 5.1 k, 1 %, 0603) | [vishay.com](https://www.vishay.com/docs/20035/dcrcwe3.pdf) |
| [yageo-cc-x7r.pdf](yageo-cc-x7r.pdf) | Yageo CC series X7R general-purpose MLCCs, 6.3 V to 250 V (CC0603KRX7R..., CC0805K...X7R...) | [yageogroup.com](https://yageogroup.com/content/datasheet/asset/file/UPY-GPHC_X7R_6_3V-TO-250V) |
| [yageo-cc0603jrnpo9bn101.pdf](yageo-cc0603jrnpo9bn101.pdf) | Yageo CC0603JRNPO9BN101 (100 pF, 50 V, C0G); the same CC NP0 series gives CC0603JRNPO9BN102 (1 nF) | [yageogroup.com](https://yageogroup.com/download/specsheet/CC0603JRNPO9BN101) |
| [kemet-c1002-x7r-smd.pdf](kemet-c1002-x7r-smd.pdf) | KEMET X7R SMD MLCCs, 6.3 V to 250 V (C1206C225K5RACTU, 2.2 uF 50 V, 1206) | [kemet.com](https://content.kemet.com/datasheets/KEM_C1002_X7R_SMD.pdf) |
| [cl31b106klhnnne.pdf](cl31b106klhnnne.pdf) | Samsung Electro-Mechanics CL31B106KLHNNNE specification sheet (10 uF, 35 V, X7R, 1206); copy hosted by Digi-Key | [mm.digikey.com](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/8439/SpecSheet_CL31B106KLHNNN_20251215.pdf) |
| [pmeg4002ej-q.pdf](pmeg4002ej-q.pdf) | Nexperia PMEG4002EJ / PMEG4002EJ-Q 40 V, 200 mA low-VF Schottky, SOD323F (SC-90); pin 1 = cathode; D1 uses PMEG4002EJ-QX | [nexperia.com](https://assets.nexperia.com/documents/data-sheet/PMEG4002EJ-Q.pdf) |
| [2n7002k.pdf](2n7002k.pdf) | Vishay Siliconix 2N7002K N-channel 60 V MOSFET, SOT-23 (pin 1 gate, 2 source, 3 drain); Q4 uses 2N7002K-T1-GE3 | [vishay.com](https://www.vishay.com/docs/71333/2n7002k.pdf) |
| [bat54c.pdf](bat54c.pdf) | Diotec BAT54 / BAT54A / BAT54C / BAT54S 30 V Schottky, SOT-23; BAT54C = common cathode on pin 3 | [diotec.com](https://diotec.com/request/datasheet/bat54.pdf) |
| [mmbt3904.pdf](mmbt3904.pdf) | Diodes Inc. MMBT3904 40 V NPN transistor, SOT23 (pin 1 base, 2 emitter, 3 collector); MMBT3904-7-F | [diodes.com](https://www.diodes.com/assets/Datasheets/MMBT3904.pdf) |
| [mmsz5246b.pdf](mmsz5246b.pdf) | Diodes Inc. MMSZ52xxB 500 mW Zener family, SOD-123; MMSZ5246B-7-F is 16 V | [diodes.com](https://www.diodes.com/assets/Datasheets/ds18010.pdf) |
| [1n4148w.pdf](1n4148w.pdf) | Diodes Inc. BAV16W / 1N4148W 100 V switching diode, SOD-123; 1N4148W-7-F | [diodes.com](https://www.diodes.com/assets/Datasheets/BAV16W_1N4148W.pdf) |
| [smbj15a.pdf](smbj15a.pdf) | Bourns SMBJ series 600 W TVS diodes, DO-214AA (SMB); SMBJ15A unidirectional, 15 V standoff | [bourns.com](https://www.bourns.com/docs/Product-Datasheets/SMBJ.pdf) |
| [sf-1206f.pdf](sf-1206f.pdf) | Bourns SF-1206F fast-acting 1206 chip fuses; SF-1206F300-2 = 3 A, 32 V DC, 50 A interrupt (F2 before change 8; kept as the record behind that change) | [bourns.com](https://www.bourns.com/docs/product-datasheets/sf-1206f.pdf) |
| [jst-ph.pdf](jst-ph.pdf) | JST PH connector series (2.0 mm): B2B-PH-K-S, B4B-PH-K-S, B8B-PH-K-S | [jst-mfg.com](https://www.jst-mfg.com/product/pdf/eng/ePH.pdf) |
| [jst-xh.pdf](jst-xh.pdf) | JST XH connector series (2.5 mm): B2B-XH-A, B4B-XH-A | [jst-mfg.com](https://www.jst-mfg.com/product/pdf/eng/eXH.pdf) |
| [keystone-3568.pdf](keystone-3568.pdf) | Keystone Electronics catalogue sheet K75: 3568 PCB fuseholder for MINI (ATM) blade fuses | [keyelco.com](https://www.keyelco.com/userAssets/file/K75p48.pdf) |
| [ncp18xh103f03rb.pdf](ncp18xh103f03rb.pdf) | Murata NCP18 series NTC thermistors, 0603 (NCP18XH103F03RB: 10 k, B25/50 3380 K, 1 %) | [murata.com](https://www.murata.com/-/media/webrenewal/products/thermistor/ntc/ncp/ncp18.ashx?la=ja-jp&cvid=20211118090222000000) |
| [amass-xt60pw.pdf](amass-xt60pw.pdf) | Amass spec sheet V1.2 for the XT60PW-F (battery side) and XT60PW-M (controller side) board-mount right-angle XT60s: 45 A rated, 60 A momentary, 0.5 mΩ; PCB pattern 2 × Ø2.7 mm on 7.2 mm, two 0.6 × 1.7 mm slots on 13.5 mm (J1) | Amass document, copy from [static.chipdip.ru](https://static.chipdip.ru/lib/583/DOC047583448.pdf) |
