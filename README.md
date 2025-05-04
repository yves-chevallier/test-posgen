# Générateur de trajectoire

## Mouvement Bang-Bang

Le mouvement bang-bang est un type de mouvement qui consiste à faire évoluer la position d'un objet par des sauts d'accélération. Il présente l'avantage d'être facile à calculer.

Il y a trois phases dans ce mouvement :

1. Accélération
2. Vitesse constante
3. Décélération

### Accélération (1)

$$
p(t) = \frac{1}{2} a t^2
v(t) = a t
a(t) = a_{nom}
$$

### Vitesse constante (2)

$$
p(t) = p_1 + v_1 t
v(t) = v_1
a = 0
$$

### Décélération (3)

$$
p(t) = p_2 + v_2 t - \frac{1}{2} a t^2
v(t) = v_2 - a t
a(t) = -a_{nom}
$$

## Critères de tests

1. La vitesse ne doit jamais dépasser la vitesse limite.
2. L'accélération ne doit jamais dépasser l'accélération maximale.
3. Aucun saut dans l'accélération ne doit survenir durant la phase de décélération.
4. Le sens de déplacement ne dois jamais changer durant un mouvement.
5. La vitesse doit être nulle à la fin de la trajectoire.
6. Le mouvement doit pouvoir être changé à tout moment.

## Unités

La vitesse et l'accélération n'ayant pas besoin d'une précision absolue, sont exprimées en `float` (32-bits). L'unité par défaut par seconde. L'accélération également donnée en `float` (32-bits) est exprimée en unités par séfaut par secondes au carré, enfin le Jerk est exprimé en secondes.

La position quant à elle est exprimée en format Q32.32, en unité par défaut.

L'unité par défaut recommandée est le tour pour les moteurs rotatifs et le millimètre pour les moteurs linéaires. Grace à la position générique, il vous est possible d'adapter comme vous le souhaitez.

Concernant le générateur de trajectoire, la variante prévue pour TMS320F28388D (FPU IEEE 64-bits), ne supporte qu'un mouvement maximum de 54 bits de résolution.

Prenons l'exemple d'un moteur rotatif avec un codeur de 8192 par par tour. Avec l'interpolation on peut obtenir :

$$
8192 \cdot 4 * 2^{12} = 134217728 pas par tour
$$

Un mouvement de 100 tours représente donc : 33 bits... C'est honnête.

## Installation

```bash
sudo apt update
sudo apt install pybind11-dev python3-pipx
pipx install uv
```

## Usage

```bash
uv venv
source venv/bin/activate
uv install
uv pip install -e .
uv run python test.py
```
