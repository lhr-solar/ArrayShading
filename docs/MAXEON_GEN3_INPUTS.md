# Maxeon Gen III cell-model inputs

The machine-readable starting dataset is `data/cells/maxeon_gen3_125mm.json`.

## Identification required

The public 125 mm Maxeon Gen III sheet defines three bins: Me1, Le1, and Ke1. The simulator
must not assume Me1 merely because it is the highest-performing bin. Confirm the marking,
purchase documentation, or flash-test record for the cells the team owns. A later 166 mm
product also carries the Gen III name and is not interchangeable with this dataset.

## Published STC values

STC is 1000 W/m², AM1.5G, and 25°C cell temperature.

| Bin | Pmpp | Efficiency | Vmpp | Impp | Voc | Isc |
|---|---:|---:|---:|---:|---:|---:|
| Me1 | 3.72 W | 24.3% | 0.632 V | 5.89 A | 0.730 V | 6.18 A |
| Le1 | 3.63 W | 23.7% | 0.621 V | 5.84 A | 0.721 V | 6.15 A |
| Ke1 | 3.54 W | 23.1% | 0.612 V | 5.79 A | 0.713 V | 6.11 A |

The sheet gives approximately 153 cm² active area, 125 mm bounding dimensions, 160 mm
diagonal, 150 ± 30 μm thickness, and 6.5 g cell mass. It lists module-context temperature
coefficients of −1.74 mV/°C voltage, +2.9 mA/°C current, and −0.29%/°C power.

## How these values should enter the first electrical model

At irradiance `G` and cell temperature `T`, the first implementation can scale short-circuit
current from STC and apply the published temperature terms:

```text
Isc(G,T) = Isc_STC (G / 1000) + alpha_I (T - 25)
Voc(G,T) = Voc_STC + beta_V (T - 25) + nVt ln(G / 1000)
Pmpp(G,T) ≈ Pmpp_STC (G / 1000) [1 + gamma_P (T - 25)]
```

The logarithmic voltage term requires a fitted diode parameter and should not be guessed.
Until that fit exists, the power-temperature approximation is appropriate for array-layout
screening but not for final string/mismatch predictions.

## Measurements needed from the actual cells

For a defensible single-diode and partial-shading model, obtain flash-test I-V curves at
multiple irradiances and temperatures. Fit `Iph`, `I0`, `n`, `Rs`, and `Rsh`; measure reverse
bias before modeling shaded series strings; record interconnect resistance; and document the
actual string and bypass-diode topology. Optical transmission and incidence-angle response
must be measured for the vehicle's encapsulation stack rather than taken from the bare-cell
sheet.

## Sources

- SunPower, *Maxeon Gen III Solar Cells*, document 507816 Rev F, mirrored by ENF Solar:
  https://www.enfsolar.com/Product/pdf/Cell/5b91fcf3916df.pdf
- Maxeon technical-document library for current module documentation:
  https://www.maxeon.com/technical-documents
