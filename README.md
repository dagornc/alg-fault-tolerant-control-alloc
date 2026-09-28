# ALG_FAULT_TOLERANT_CONTROL_ALLOC — Portage Rust

Implémentation Rust de référence de l'algorithme **ALG_FAULT_TOLERANT_CONTROL_ALLOC**
(allocation de commande tolérante aux fautes), avec une référence Python normative
en miroir et un harnais de vérification de parité bit-à-bit.

## Ce qui est implémenté

- **Modèle simulé** : allocation pondérée d'une consigne 3-DOF (surge, sway, yaw)
  vers **quatre actionneurs** sur-actionnés (port, tribord, étrave latérale, poupe
  latérale), via une matrice d'allocation fixe.
- **Solveur** : équations normales 3×3 résolues par élimination de Gauss avec pivot
  partiel déterministe, régularisation de Tikhonov (1e-9), puis **projection active
  des saturations** (clamp itératif de l'actionneur le plus saturé).
- **Pondération par santé** : un actionneur dégradé (santé < 1) est pénalisé dans la
  norme pondérée ; un actionneur en faute (santé = 0) est exclu et reçoit une
  commande nulle.
- **Scénarios** : `nominal`, `port_failure`, `bow_degraded`, `dual_fault`,
  `saturation`.
- **Métriques** : erreur de commande résiduelle (RMS), événements de saturation,
  saturation évitée, enveloppe de manœuvre restante, nombre d'actionneurs en faute.

## Ce qui est vérifié

- `cargo build` (debug et release) : **OK**.
- `cargo test` : **12 tests passants** (4 unitaires RNG + 8 d'intégration).
- `cargo clippy --all-targets -- -D warnings` : **OK** (aucun avertissement).
- Parité Rust ↔ Python : **250/250 comparaisons identiques, 0 écart** sur
  5 scénarios × 50 graines (1000–1049), 64 pas chacune.

  Rejeu :

  ```sh
  cargo build --release
  python3 verify_parite_rust.py
  ```

  Statut de parité : **bit-à-bit VÉRIFIÉE**.

## Ce qui ne l'est pas

- **Simplifications scientifiques** : modèle cinématique statique (pas de dynamique
  de propulseur, pas d'identification système, pas de validation HITL). La source
  de vérité (10.1016/j.oceaneng.2026.127394) décrit une allocation hybride avec
  saturation azimuth et validation HITL ; le portage se limite à la variante
  « pseudo-inverse pondérée + projection de saturation ».
- **Hypothèse** : la faute est détectée et qualifiée en amont (par
  `ALG_HEALTH_MONITORING`) ; ce portage ne fait pas de détection de faute.
- **Non couvert** : allocation hybride, allocation sous saturation azimuth,
  intégration au fil de sûreté UAV (voilure fixe/tournante).

## Structure

```
Cargo.toml
src/rng.rs            RNG MT19937 compatible CPython (copié du socle RUST-13)
src/sim.rs            simulateur déterministe + solveur d'allocation
src/lib.rs            API publique
src/main.rs           CLI CSV
tests/parite.rs       tests d'intégration + invariants
reference/sim_fault_tolerant_control_alloc.py   référence Python normative
verify_parite_rust.py harnais de parité
```

## Usage

```sh
cargo run --release -- 1001 nominal 64
# seed,scenario,steps,residual_rms,saturation_events,saturation_avoided,maneuver_envelope,failed_actuators
```

## Licence

MIT — Christophe Dagorn.
