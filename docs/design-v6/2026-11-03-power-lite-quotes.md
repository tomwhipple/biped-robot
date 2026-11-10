# Power lite: board quotes (2026-11-03)

*Bare boards for [pcb/power-lite](../../pcb/power-lite/README.md) rev A: 2 layers, 97 × 64.5 mm,
1.6 mm FR4, 1 oz copper, green mask, through-hole only. Prices are each fab's public instant
quote, read today without an account or an upload, shipped to the US.*

| fab | boards | board | shipping | duty and fees | landed | time |
|---|---|---|---|---|---|---|
| **JLCPCB** (China) | 5 | $4.00 | $26.86, UPS Express Saver DDP | about $1.40, prepaid (35 % of $4) | **about $32.26** | 3-day build, 2–4 business days |
| **PCBWay** (China) | 5 | $5.00 | $26.55, DHL | about $1.75 duty, plus DHL's $17.50 duty-processing fee, paid on delivery | **about $50.80** | 24 h build, 2–4 business days |
| **OSH Park** (US) | 3 | $48.49 | free | none | **$48.49** | 9–12 calendar days |

- **The other quantities and options:**
  - PCBWay: 10 boards cost $19.22 + $26.55 DHL = $45.77 before duty. 5 boards in 2 oz copper
    cost $43.10 + $26.55 = $69.65, with a 3–4 day build.
  - JLCPCB's price is $2.00 when ordered through its desktop app. Its default finish is HASL
    with lead; PCBWay's quote is lead-free HASL. OSH Park's boards are ENIG with purple mask.
  - OSH Park sells sets of 3 at $5 a square inch: 9.70 sq in is $48.49, and 6 boards are
    $96.97.
- **Duty:**
  - JLCPCB ships to individuals DDP. It prepays US duty on PCBs at 35 %, from 2026-03-17, and
    refunds or charges the difference after the carrier's bill.
  - PCBWay's quote says "Customs duties and VAT are not included!", so DHL bills them on delivery.
    The 35 % is JLCPCB's rate for the same goods.
  - DHL's US fee for paying duty for the receiver is a $17.50 minimum or 2 %. That it applies
    to this shipment is inferred, not quoted.
- **1 oz is enough.** The narrowest section of the 15 A path is 6.1 mm of copper with the GND
  plane beneath it, and the return pads join the plane with full copper (the board README's
  Checks). 2 oz would double PCBWay's price and is not offered at 1.6 mm by OSH Park.
- **Accounts:** PCBWay already has one; JLCPCB and OSH Park would need one.

## Sources

- PCBWay's [instant quote](https://www.pcbway.com/orderonline.aspx), run with `quote2l.py`
  (2 layers, 97 × 65 mm, 1.6 mm, 8/8 mil, lead-free HASL, green, DHL to the US).
- JLCPCB's [quote page](https://cart.jlcpcb.com/quote), with 97 × 64.5 mm, 5 pieces and 1.6 mm
  set and everything else left at its default. The shipping estimate is from the page's
  `calculateGoodsFreight` response, country US.
- JLCPCB, [US tariff policy updates](https://jlcpcb.com/blog/logistics-policy-updates-due-to-the-us-tariff-on-china)
  (PCBs 35 %, from 2026-03-17), and the [US tariff FAQ](https://jlcpcb.com/help/article/us-tariff-policy-faq).
- DHL Express US, [payment and billing FAQ](https://mydhl.express.dhl/us/en/help-and-support/faqs/payment-billing.html)
  (Duty Tax Processing, $17.50 minimum or 2 %, from 2026-01-01).
- OSH Park, [fabrication services](https://docs.oshpark.com/services/), and
  [all boards are manufactured in the United States](https://blog.oshpark.com/2020/03/04/all-boards-are-manufactured-in-the-united-states/).
