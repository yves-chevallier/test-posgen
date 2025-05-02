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
