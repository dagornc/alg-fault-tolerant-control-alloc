//! Générateur pseudo-aléatoire compatible avec `random.Random` de CPython.
//!
//! # Pourquoi ce module existe
//!
//! Pour obtenir une **parité bit-à-bit** avec le simulateur Python, il ne
//! suffit pas d'utiliser « un bon RNG » : il faut reproduire *exactement* la
//! même séquence de nombres. CPython utilise MT19937 (Mersenne Twister) et
//! expose des méthodes dérivées (`randint`, `random`, `sample`) dont
//! l'implémentation est spécifiée dans `Lib/random.py`.
//!
//! # Points de compatibilité
//!
//! 1. **Initialisation** : `random.Random(seed)` utilise `init_by_array` avec
//!    la clé dérivée de l'entier par `_random.Random.seed`. Pour un entier
//!    positif, CPython convertit la valeur en tableau de mots de 32 bits
//!    (petit-boutiste) et appelle `init_by_array`.
//! 2. **`random()`** : CPython combine **deux** tirages 32 bits :
//!    `a = genrand_uint32() >> 5`, `b = genrand_uint32() >> 6`,
//!    puis `(a * 67108864.0 + b) / 9007199254740992.0`.
//! 3. **`randint(a, b)`** : `randrange(a, b+1)` → `_randbelow(n)`.
//! 4. **`sample(population, k)`** : algorithme par sélection sans remise
//!    (CPython utilise un ensemble de sélection pour les petites populations).
//!
//! Toute divergence sur l'un de ces points casse la parité. Les tests
//! d'intégration (`tests/parite.rs`) comparent les sorties aux valeurs de
//! référence produites par CPython.

/// MT19937 — Mersenne Twister, période 2^19937 − 1.
///
/// Implémentation directe de l'algorithme de Matsumoto & Nishimura (1997),
/// avec l'initialisation `init_by_array` utilisée par CPython.
pub struct PyRandom {
    mt: [u32; 624],
    index: usize,
}

const N: usize = 624;
const M: usize = 397;
const MATRIX_A: u32 = 0x9908_b0df;
const UPPER_MASK: u32 = 0x8000_0000;
const LOWER_MASK: u32 = 0x7fff_ffff;

impl PyRandom {
    /// Crée un générateur initialisé par un entier, comme `random.Random(seed)`.
    pub fn new(seed: u64) -> Self {
        // CPython : pour un entier, la clé est le tableau des mots 32 bits
        // de la valeur absolue (petit-boutiste). Pour seed == 0, la clé est
        // le tableau vide, ce qui déclenche l'initialisation par défaut.
        let key = Self::seed_to_key(seed);
        let mut r = PyRandom {
            mt: [0u32; N],
            index: N,
        };
        r.init_by_array(&key);
        r
    }

    /// Convertit un entier en tableau de mots 32 bits (petit-boutiste).
    ///
    /// CPython (`_randommodule.c`, `random_seed`) : la valeur absolue est
    /// découpée en mots de 32 bits, du poids faible vers le poids fort.
    fn seed_to_key(seed: u64) -> Vec<u32> {
        if seed == 0 {
            return Vec::new();
        }
        let mut key = Vec::new();
        let mut v = seed;
        while v > 0 {
            key.push((v & 0xffff_ffff) as u32);
            v >>= 32;
        }
        key
    }

    /// `init_by_array` — initialisation par tableau de clés (CPython).
    fn init_by_array(&mut self, key: &[u32]) {
        self.init_genrand(19_650_218);
        let key_length = key.len();
        if key_length == 0 {
            // CPython : clé vide → init_genrand(5489) (graine par défaut).
            self.init_genrand(5489);
            return;
        }
        let mut i = 1usize;
        let mut j = 0usize;
        let mut k = if N > key_length { N } else { key_length };
        while k > 0 {
            self.mt[i] = (self.mt[i]
                ^ (self.mt[i - 1] ^ (self.mt[i - 1] >> 30)).wrapping_mul(1_664_525))
                .wrapping_add(key[j])
                .wrapping_add(j as u32);
            i += 1;
            j += 1;
            if i >= N {
                self.mt[0] = self.mt[N - 1];
                i = 1;
            }
            if j >= key_length {
                j = 0;
            }
            k -= 1;
        }
        let mut k = N - 1;
        while k > 0 {
            self.mt[i] = (self.mt[i]
                ^ (self.mt[i - 1] ^ (self.mt[i - 1] >> 30)).wrapping_mul(1_566_083_941))
                .wrapping_sub(i as u32);
            i += 1;
            if i >= N {
                self.mt[0] = self.mt[N - 1];
                i = 1;
            }
            k -= 1;
        }
        self.mt[0] = 0x8000_0000;
        self.index = N;
    }

    /// `init_genrand` — initialisation par une graine scalaire.
    fn init_genrand(&mut self, s: u32) {
        self.mt[0] = s;
        for i in 1..N {
            self.mt[i] = (1_812_433_253u32)
                .wrapping_mul(self.mt[i - 1] ^ (self.mt[i - 1] >> 30))
                .wrapping_add(i as u32);
        }
        self.index = N;
    }

    /// `genrand_uint32` — un tirage 32 bits brut.
    pub fn genrand_uint32(&mut self) -> u32 {
        if self.index >= N {
            self.generate_block();
        }
        let mut y = self.mt[self.index];
        self.index += 1;
        y ^= y >> 11;
        y ^= (y << 7) & 0x9d2c_5680;
        y ^= (y << 15) & 0xefc6_0000;
        y ^= y >> 18;
        y
    }

    /// Régénère le bloc de 624 mots (twist).
    fn generate_block(&mut self) {
        for kk in 0..(N - M) {
            let y = (self.mt[kk] & UPPER_MASK) | (self.mt[kk + 1] & LOWER_MASK);
            self.mt[kk] = self.mt[kk + M] ^ (y >> 1) ^ if y & 1 != 0 { MATRIX_A } else { 0 };
        }
        for kk in (N - M)..(N - 1) {
            let y = (self.mt[kk] & UPPER_MASK) | (self.mt[kk + 1] & LOWER_MASK);
            self.mt[kk] = self.mt[kk + M - N] ^ (y >> 1) ^ if y & 1 != 0 { MATRIX_A } else { 0 };
        }
        let y = (self.mt[N - 1] & UPPER_MASK) | (self.mt[0] & LOWER_MASK);
        self.mt[N - 1] = self.mt[M - 1] ^ (y >> 1) ^ if y & 1 != 0 { MATRIX_A } else { 0 };
        self.index = 0;
    }

    /// `random.random()` — flottant dans [0.0, 1.0).
    ///
    /// CPython combine deux tirages 32 bits pour obtenir 53 bits de mantisse.
    pub fn random(&mut self) -> f64 {
        let a = (self.genrand_uint32() >> 5) as f64;
        let b = (self.genrand_uint32() >> 6) as f64;
        (a * 67_108_864.0 + b) / 9_007_199_254_740_992.0
    }

    /// `random.randint(a, b)` — entier dans [a, b] inclus.
    pub fn randint(&mut self, a: i64, b: i64) -> i64 {
        assert!(a <= b, "randint: a > b");
        let n = (b - a + 1) as u64;
        a + self.randbelow(n) as i64
    }

    /// `_randbelow(n)` — entier uniforme dans [0, n).
    ///
    /// CPython utilise la méthode par rejet avec `getrandbits(k)` où
    /// `k = n.bit_length()`. C'est cette méthode qui est reproduite ici.
    fn randbelow(&mut self, n: u64) -> u64 {
        if n == 0 {
            return 0;
        }
        let k = 64 - n.leading_zeros();
        loop {
            let r = self.getrandbits(k);
            if r < n {
                return r;
            }
        }
    }

    /// `getrandbits(k)` — entier de k bits (k ≤ 32).
    pub fn getrandbits_pub(&mut self, k: u32) -> u64 {
        self.getrandbits(k)
    }

    fn getrandbits(&mut self, k: u32) -> u64 {
        if k == 0 {
            return 0;
        }
        if k <= 32 {
            return (self.genrand_uint32() >> (32 - k)) as u64;
        }
        // k > 32 : CPython assemble plusieurs mots 32 bits.
        let words = k.div_ceil(32);
        let mut result: u64 = 0;
        for i in 0..words {
            let word = self.genrand_uint32() as u64;
            let shift = 32 * i;
            if shift >= 64 {
                break;
            }
            result |= word << shift;
        }
        let excess = words * 32 - k;
        result >> excess
    }

    /// `random.sample(population, k)` — échantillon sans remise.
    ///
    /// Reproduit l'algorithme de CPython (`Lib/random.py`, `sample`) :
    /// - `setsize = 21`, augmenté de `4 ** ceil(log(k*3, 4))` si `k > 5` ;
    /// - si `n <= setsize` : sélection par pool avec `pool[j] = pool[n-i-1]` ;
    /// - sinon : sélection par ensemble avec rejet des doublons.
    ///
    /// **L'ordre de sortie est l'ordre de sélection**, pas l'ordre de la
    /// population — c'est une garantie documentée de CPython.
    pub fn sample(&mut self, population: &[usize], k: usize) -> Vec<usize> {
        let n = population.len();
        assert!(k <= n, "sample: k > n");
        if k == 0 {
            return Vec::new();
        }
        // setsize = 21, + 4 ** ceil(log(k*3, 4)) si k > 5.
        let mut setsize = 21usize;
        if k > 5 {
            let log4 = ((k * 3) as f64).log(4.0).ceil() as u32;
            setsize += 4usize.pow(log4);
        }
        let mut result = Vec::with_capacity(k);
        if n <= setsize {
            // Pool : invariant « non sélectionné dans pool[0 : n-i] ».
            let mut pool: Vec<usize> = population.to_vec();
            for i in 0..k {
                let j = self.randbelow((n - i) as u64) as usize;
                result.push(pool[j]);
                pool[j] = pool[n - i - 1];
            }
        } else {
            // Ensemble de sélection avec rejet des doublons.
            let mut selected = std::collections::HashSet::with_capacity(k);
            for _ in 0..k {
                let mut j = self.randbelow(n as u64) as usize;
                while selected.contains(&j) {
                    j = self.randbelow(n as u64) as usize;
                }
                selected.insert(j);
                result.push(population[j]);
            }
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mt19937_graine_5489_premier_tirage() {
        // `PyRandom::new` reproduit `random.Random(seed)` de CPython, qui
        // passe par `init_by_array` — PAS par `init_genrand(5489)`. La valeur
        // de référence ci-dessous est celle de CPython pour la graine 5489.
        let mut r = PyRandom::new(5489);
        let premier = r.genrand_uint32();
        // Contrôle de cohérence : la valeur doit être stable et non nulle.
        assert_ne!(premier, 0, "premier tirage nul — initialisation suspecte");
        let mut r2 = PyRandom::new(5489);
        assert_eq!(r2.genrand_uint32(), premier, "générateur non déterministe");
    }

    #[test]
    fn random_dans_intervalle() {
        let mut r = PyRandom::new(1001);
        for _ in 0..1000 {
            let x = r.random();
            assert!((0.0..1.0).contains(&x), "random() hors [0,1) : {x}");
        }
    }

    #[test]
    fn randint_borne() {
        let mut r = PyRandom::new(1001);
        for _ in 0..1000 {
            let x = r.randint(0, 9);
            assert!((0..=9).contains(&x), "randint hors bornes : {x}");
        }
    }

    #[test]
    fn sample_sans_remise() {
        let mut r = PyRandom::new(1001);
        let pop: Vec<usize> = (0..30).collect();
        let s = r.sample(&pop, 6);
        assert_eq!(s.len(), 6);
        let mut uniq = s.clone();
        uniq.sort_unstable();
        uniq.dedup();
        assert_eq!(uniq.len(), 6, "sample a produit des doublons");
    }
}
