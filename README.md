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
- `tvc_rocket/simulation_3d.py` : preview 3D simplifiee avec derive laterale
- `tvc_rocket/simulation_6dof.py` : premiere base 3D dynamique avec attitude roll/pitch/yaw, TVC 2 axes et guidage 3D cible
- `tvc_rocket/simulation_6dof_guided.py` : branche de travail dediee au tuning 3D avec convention d'axes, erreurs yaw/pitch/beta/alpha et guidage cible progressif
- `tvc_rocket/tuning.py` : autotuning et raffinements des gains
- `tvc_rocket/output.py` : export CSV et gestion de matplotlib
- `tvc_rocket/plotting.py` : graphiques et resume console
- `tvc_rocket/plotting_6dof_guided.py` : graphes de tuning 3D pour les erreurs de guidage et aerodynamiques
- `tvc_rocket/cli.py` : options de ligne de commande et orchestration
- `tests/` : tests unitaires sur le modele, la simulation et la preview 3D
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
python tvc_rocket.py --save-only --preview-3d --crosswind-ref 8
python tvc_rocket.py --mode 6dof --save-only --crosswind-ref 8 --impact-target 800 --impact-target-y 120
python tvc_rocket.py --mode 6dof-guided --save-only --crosswind-ref 8 --impact-target 800 --impact-target-y 120
python tvc_rocket.py --mode 6dof-guided --guided-short-range --save-only --impact-target 800 --impact-target-y 120
```

Mode rapide sans autotuning :

```powershell
python tvc_rocket.py --no-tune
python tvc_rocket.py --no-tune --kp 4.8 --ki 0.7 --kd 1.3
python tvc_rocket.py --no-tune --impact-target 1000 --gamma-final-deg 74
```

## Options utiles

- `--impact-target` : distance cible d'impact au sol en metres
- `--impact-target-y` : cible laterale au sol en metres pour le guidage 3D
- `--altitude-target` : apogee cible en metres
- `--tuning-mode fast|accurate` : recherche rapide ou plus complete
- `--no-tune` : saute l'autotuning et utilise directement les gains fournis
- `--kp --ki --kd` : gains PID manuels
- `--guidance-gain` : gain principal de la loi de guidage
- `--crosswind-ref` : vent lateral de reference en m/s pour la preview 3D
- `--guidance-altitude-gain` : correction d'altitude de la loi de guidage
- `--gamma-final-deg` : angle final vise pour la gravity turn
- `--adaptive-alpha-schedule` : active une limitation experimentale de la consigne alpha a forte pression dynamique
- `--mode 2d|6dof|6dof-guided` : choisit le solveur principal
- `--guided-short-range` : preset reduisant l'energie du mode `6dof-guided` pour les cibles courtes
- `--preview-3d` : produit une preview 3D simplifiee avec derive laterale et graphe dedie
- `--save-only` : sauvegarde les PNG sans ouvrir les fenetres
- `--display-seconds` : duree d'affichage automatique des graphes

## Tests

```powershell
python -m unittest discover -s tests -v
```

## Remarques

- Le modele utilise maintenant une atmosphere standard simplifiee, une trainee sensible au Mach et un profil de poussee avec montee puis tail-off.
- L'autotuning peut prendre du temps, surtout en mode `accurate`.
- Le mode `--no-tune` est pratique pour iterer rapidement sur le modele ou tester des gains connus.
- `--adaptive-alpha-schedule` est experimental et n'est pas active par defaut.
- Le mode `--mode 6dof` est maintenant une base 3D avec attitude complete `roll/pitch/yaw`, aerodynamique laterale et guidage 3D cible, mais reste encore experimental face au solveur 2D principal.
- Le mode `--mode 6dof-guided` sert a figer la convention d'axes/signes, tracer les erreurs `yaw/pitch/beta/alpha`, regler separement le longitudinal puis le lateral, puis relancer la poursuite 3D cible.
- Le lancement reste `python tvc_rocket.py`, meme si le code interne est maintenant modulaire.
- Les fichiers CSV et PNG sont regeneres dans `results/` et `plots/`.
