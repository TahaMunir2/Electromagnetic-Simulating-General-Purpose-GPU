import math

N_PML  = 6
m      = 2
SCALE  = 2**13

C_E_fp = 717
C_B_fp = 2867

C_E = C_E_fp / SCALE
C_B = C_B_fp / SCALE
dt  = C_E

# sigma_max chosen so ca at the outermost cell is around 0.15
target_ca = 0.15
x = (1 - target_ca) / (1 + target_ca)
sigma_max = 2 * x / dt

print(f"sigma_max = {sigma_max:.4f}")
print()

for d in range(N_PML):
    sigma    = sigma_max * ((d / N_PML) ** m)
    denom    = 1 + sigma * dt / 2
    ca       = (1 - sigma * dt / 2) / denom
    cb_e     = -C_E / denom
    cb_bz    = -C_B / denom

    ca_fp    = round(ca   * SCALE)
    cb_e_fp  = round(cb_e * SCALE)
    cb_bz_fp = round(cb_bz * SCALE)

    print(f"d={d}: ca={ca_fp}, cb_e={cb_e_fp}, cb_bz={cb_bz_fp}")
