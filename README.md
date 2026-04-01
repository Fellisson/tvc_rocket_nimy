# TVC Rocket Nimy

Simulation Python d'une fusee a poussee vectorielle (`TVC`) avec :

- guidage automatique vers un point d'impact cible ;
- regulateur PID pour le controle d'attitude ;
- tuning automatique des gains ;
- export des resultats en CSV ;
- generation automatique des graphes en PNG.

## Structure

- `tvc_rocket.py` : point d'entree conserve pour lancer le projet simplement
- `tvc_rocket/models.py` : parametres, etats, gains et utilitaires de base
- `tvc_rocket/simulation.py` : dynamique, simulation et evaluation de trajectoire
- `tvc_rocket/tuning.py` : autotuning et raffinements des gains
- `tvc_rocket/output.py` : export CSV et gestion de matplotlib
- `tvc_rocket/plotting.py` : graphiques et resume console
- `tvc_rocket/cli.py` : options de ligne de commande et orchestration
- `results/` : sorties CSV de la meilleure simulation et du tuning
- `plots/` : figures generees par le script

## Installation

```powershell
python -m pip install -r requirements.txt
```

## Lancement

Mode complet avec autotuning :

```powershell
python tvc_rocket.py
python tvc_rocket.py --impact-target 1500
python tvc_rocket.py --impact-target 800 --tuning-mode fast
python tvc_rocket.py --save-only
```

Mode rapide sans autotuning :

```powershell
python tvc_rocket.py --no-tune
python tvc_rocket.py --no-tune --kp 4.8 --ki 0.7 --kd 1.3
python tvc_rocket.py --no-tune --impact-target 1000 --gamma-final-deg 74
```

## Options utiles

- `--impact-target` : distance cible d'impact au sol en metres
- `--altitude-target` : apogee cible en metres
- `--tuning-mode fast|accurate` : recherche rapide ou plus complete
- `--no-tune` : saute l'autotuning et utilise directement les gains fournis
- `--kp --ki --kd` : gains PID manuels
- `--guidance-gain` : gain principal de la loi de guidage
- `--guidance-altitude-gain` : correction d'altitude de la loi de guidage
- `--gamma-final-deg` : angle final vise pour la gravity turn
- `--adaptive-alpha-schedule` : active une limitation experimentale de la consigne alpha a forte pression dynamique
- `--save-only` : sauvegarde les PNG sans ouvrir les fenetres
- `--display-seconds` : duree d'affichage automatique des graphes

## Remarques

- L'autotuning peut prendre du temps, surtout en mode `accurate`.
- Le mode `--no-tune` est pratique pour iterer rapidement sur le modele ou tester des gains connus.
- `--adaptive-alpha-schedule` est experimental et n'est pas active par defaut.
- Le lancement reste `python tvc_rocket.py`, meme si le code interne est maintenant modulaire.
- Les fichiers CSV et PNG sont regeneres dans `results/` et `plots/`.
