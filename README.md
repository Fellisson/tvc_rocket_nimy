# TVC Rocket Nimy

Simulation Python d'une fusee a poussée vectorielle (`TVC`) avec:

- guidage automatique vers un point d'impact cible
- regulateur PID pour le controle d'attitude
- tuning automatique des gains
- export des resultats en CSV
- generation automatique des graphes en PNG

## Structure

- `tvc_rocket.py` : script principal
- `results/` : sorties CSV de la meilleure simulation et du tuning
- `plots/` : figures generees par le script

## Lancement

```powershell
python tvc_rocket.py
python tvc_rocket.py --impact-target 1500
python tvc_rocket.py --impact-target 800 --tuning-mode fast
python tvc_rocket.py --save-only
```

## Options utiles

- `--impact-target` : distance cible d'impact au sol en metres
- `--altitude-target` : apogee cible en metres
- `--tuning-mode fast|accurate` : recherche rapide ou plus complete
- `--save-only` : sauvegarde les PNG sans ouvrir les fenetres
- `--display-seconds` : duree d'affichage automatique des graphes

## Fichiers inclus

Le depot contient un jeu de resultats genere a partir de la configuration actuelle du script:

- `results/tvc_rocket_best_history.csv`
- `results/tvc_rocket_tuning_results.csv`
- `plots/tvc_rocket_main_plots.png`
- `plots/tvc_rocket_top5_comparison.png`
- `plots/tvc_rocket_wind_profile.png`
