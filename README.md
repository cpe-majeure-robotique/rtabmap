rtabmap
=======

## À propos de ce fork CPE

Ce dépôt est le fork CPE de
[`introlab/rtabmap`](https://github.com/introlab/rtabmap). La branche utilisée
par le Crawler est `CuVSLAM_15_support`. Elle conserve les bibliothèques, outils
et applications RTAB-Map d'origine et adapte son backend d'odométrie NVIDIA à
cuVSLAM 15.

Principales différences avec la source IntRoLab à la révision de départ
`468f6e6f` :

- migration de l'ancienne API C cuVSLAM 14 vers l'API C++ cuVSLAM 15 ;
- prise en charge des entrées stéréo, multicaméra et RGB-D ;
- conversion des repères mise à jour pour les coordonnées optiques de v15 ;
- export des observations et landmarks vers les informations d'odométrie ;
- validation, bornage, mise à l'échelle et lissage de la covariance cuVSLAM ;
- test unitaire du traitement de covariance ;
- runtimes cuVSLAM 15 embarqués pour CUDA 12/13, Ubuntu 22.04/24.04 et
  `aarch64`/`x86_64` ;
- sélection automatique du runtime, installation de ses en-têtes/licence et
  conservation des RPATH de ses dépendances ;
- exclusion des artefacts de compilation produits directement dans le dépôt.

Les binaires cuVSLAM restent soumis à la licence NVIDIA fournie dans
[`third_party/cuvslam/15.0.0/LICENSE`](third_party/cuvslam/15.0.0/LICENSE). Leur
provenance et leurs sommes SHA-256 sont documentées dans
[`third_party/cuvslam/15.0.0`](third_party/cuvslam/15.0.0/README.md).

## Fonctionnalités

RTAB-Map fournit une bibliothèque C++ de SLAM à détection de fermetures de
boucle, cartographie métrique et odométrie visuelle/lidar, ainsi qu'une
application autonome et des outils de manipulation de bases `.db`. Dans le
Crawler, ce dépôt fournit surtout la bibliothèque chargée par `rtabmap_ros` et
le backend `OdometryCuVSLAM` accéléré par CUDA.

Le mode cuVSLAM accepte :

- une ou plusieurs paires stéréo rectifiées ;
- une caméra RGB-D avec image couleur et profondeur ;
- les contraintes planes optionnelles ;
- trois compromis multicaméra : modéré, performance et précision.

## Compilation du fork

Configuration recommandée pour le Crawler :

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DWITH_CUVSLAM=ON
cmake --build build -j
sudo cmake --install build
```

CUDA doit être présent sur la machine. CMake choisit ensuite le runtime
embarqué correspondant à CUDA, Ubuntu et l'architecture détectés.

| Option CMake | Défaut | Description |
|---|---:|---|
| `WITH_CUVSLAM` | `OFF` | Compile le backend d'odométrie cuVSLAM. |
| `CUVSLAM_USE_BUNDLED` | `ON` | Utilise les runtimes embarqués si aucun chemin externe n'est imposé. |
| `CUVSLAM_ROOT_DIR` | vide | Racine d'une installation cuVSLAM externe, prioritaire sur le runtime embarqué. Les variables d'environnement `CUVSLAM_ROOT_DIR` et `CUVSLAM_ROOT` sont aussi reconnues. |
| `CUVSLAM_BUNDLED_CUDA_VERSION` | auto | Force le runtime CUDA `12` ou `13`. |
| `CUVSLAM_BUNDLED_UBUNTU_VERSION` | auto | Force Ubuntu `22` ou `24`. |
| `CUVSLAM_BUNDLED_ARCH` | auto | Force `aarch64` ou `x86_64`. |
| `BUILD_WITH_RPATH_NOT_RUNPATH` | `OFF` | Demande l'ancien comportement RPATH pour l'arbre de compilation. |

Une combinaison embarquée inexistante déclenche une recherche de cuVSLAM sur
le système. Le résumé CMake doit afficher `With cuVSLAM = YES` avant de lancer
la compilation.

## Paramètres d'exécution cuVSLAM

Ces paramètres RTAB-Map sont accessibles via le fichier `.ini`, les arguments
de l'application et les wrappers ROS sous leur nom complet.

| Paramètre | Défaut | Description |
|---|---:|---|
| `OdomCuVSLAM/MulticamMode` | `0` | `0` modéré, `1` performance, `2` précision. |
| `OdomCuVSLAM/UseRawCovariance` | `false` | Transmet la covariance assainie sans calibration d'échelle ni lissage des baisses. |
| `OdomCuVSLAM/CovariancePositionScale` | `1.0` | Facteur appliqué aux variances de translation. |
| `OdomCuVSLAM/CovarianceOrientationScale` | `1.0` | Facteur appliqué aux variances de rotation. |
| `OdomCuVSLAM/CovariancePositionFloor` | `1e-6` m² | Plancher de variance de position. |
| `OdomCuVSLAM/CovarianceOrientationFloor` | `1e-6` rad² | Plancher de variance d'orientation. |
| `OdomCuVSLAM/CovariancePositionCeiling` | `100.0` m² | Plafond numérique de variance de position. |
| `OdomCuVSLAM/CovarianceOrientationCeiling` | `9.8696` rad² | Plafond numérique de variance d'orientation. |
| `OdomCuVSLAM/CovarianceFallbackPosition` | `0.25` m² | Valeur conservative si cuVSLAM renvoie une variance invalide. |
| `OdomCuVSLAM/CovarianceFallbackOrientation` | `0.1` rad² | Repli équivalent pour l'orientation. |
| `OdomCuVSLAM/CovarianceDecreaseSmoothing` | `0.9` | Lissage unilatéral `[0,1[` : les hausses restent immédiates, les baisses sont amorties ; `0` désactive le lissage. |
| `OdomCuVSLAM/MinLandmarks` | `0` | Nombre minimal de landmarks à l'initialisation ; `0` fait confiance à toute pose cuVSLAM valide. |

Les autres paramètres génériques (`Odom/*`, `Reg/*`, `Vis/*`, `RGBD/*`,
`Grid/*`, etc.) restent ceux de RTAB-Map amont. Les profils
[`data/presets/lidar3d_icp.ini`](data/presets/lidar3d_icp.ini) et
[`data/presets/camera_tof_icp.ini`](data/presets/camera_tof_icp.ini) servent de
bases pour les chaînes ICP ; les valeurs spécifiques au robot sont conservées
dans `crawler_nav`.

## Exécutables installés

| Groupe | Exécutables | Usage |
|---|---|---|
| Application | `rtabmap` | Interface autonome de cartographie et inspection. |
| Acquisition | `rtabmap-camera`, `rtabmap-rgbd_camera`, `rtabmap-dataRecorder`, `rtabmap-calibration` | Caméras, enregistrement et calibration. |
| Odométrie/visualisation | `rtabmap-odometryViewer`, `rtabmap-lidar_viewer` | Diagnostic interactif de l'odométrie et des nuages lidar. |
| Base et graphe | `rtabmap-databaseViewer`, `rtabmap-info`, `rtabmap-reprocess`, `rtabmap-recovery`, `rtabmap-reduceGraph`, `rtabmap-detectMoreLoopClosures`, `rtabmap-cleanupLocalGrids`, `rtabmap-globalBundleAdjustment` | Inspection, récupération et optimisation des bases RTAB-Map. |
| Import/export | `rtabmap-export`, `rtabmap-report`, `rtabmap-extractObject`, `rtabmap-imagesJoiner` | Export de cartes/données, rapports et traitement d'images/objets. |
| Jeux de données | `rtabmap-kitti_dataset`, `rtabmap-euroc_dataset`, `rtabmap-rgbd_dataset`, `rtabmap-cidsims_dataset` | Lecture et évaluation de datasets. |
| Analyse | `rtabmap-matcher`, `rtabmap-stereoEval`, `rtabmap-epipolar_geometry`, `rtabmap-vocabularyComparison` | Appariement, géométrie et comparaison de vocabulaires. |
| Ligne de commande | `rtabmap-console` | Exécution sans interface graphique. |

La disponibilité exacte dépend des bibliothèques détectées à la compilation.
Utiliser `<exécutable> --help` pour les arguments CLI générés par la version
construite. Les programmes de `examples/` sont des démonstrateurs de
développement et ne font pas partie de l'interface stable du Crawler.

[![RTAB-Map Logo](https://raw.githubusercontent.com/introlab/rtabmap/master/guilib/src/images/RTAB-Map100.png)](http://introlab.github.io/rtabmap)

[![Release][release-image]][releases]
[![Downloads][downloads-image]][downloads]
[![License][license-image]][license]

[release-image]: https://img.shields.io/badge/release-0.23.1-green.svg?style=flat
[releases]: https://github.com/introlab/rtabmap/releases

[downloads-image]: https://img.shields.io/github/downloads/introlab/rtabmap/total?label=downloads
[downloads]: https://github.com/introlab/rtabmap/releases

[license-image]: https://img.shields.io/badge/license-BSD-green.svg?style=flat
[license]: https://github.com/introlab/rtabmap/blob/master/LICENSE

RTAB-Map library and standalone application.

 * For more information (e.g., papers, major updates), visit [RTAB-Map's home page](http://introlab.github.io/rtabmap).
 * For installation instructions and examples, visit [RTAB-Map's wiki](https://github.com/introlab/rtabmap/wiki).

To use RTAB-Map under ROS, visit the [rtabmap](http://wiki.ros.org/rtabmap) page on the ROS wiki.

### Acknowledgements
This project is supported by [IntRoLab - Intelligent / Interactive / Integrated / Interdisciplinary Robot Lab](https://introlab.3it.usherbrooke.ca/), Sherbrooke, Québec, Canada.

<a href="https://introlab.3it.usherbrooke.ca/">
<img src="https://github.com/introlab/16SoundsUSB/blob/master/images/IntRoLab.png" alt="IntRoLab" height="100">
</a>

#### CI Latest

  <table>
    <tbody>
        <tr>
           <td>
           <a href="https://github.com/introlab/rtabmap/actions/workflows/cmake-linux.yml"><img src="https://github.com/introlab/rtabmap/actions/workflows/cmake-linux.yml/badge.svg" alt="CMake Linux Build Status"/> <br> 
           <a href="https://github.com/introlab/rtabmap/actions/workflows/cmake-windows.yml"><img src="https://github.com/introlab/rtabmap/actions/workflows/cmake-windows.yml/badge.svg" alt="CMake Windows Build Status"/> <br> 
           <a href="https://github.com/introlab/rtabmap/actions/workflows/cmake-macos.yml"><img src="https://github.com/introlab/rtabmap/actions/workflows/cmake-macos.yml/badge.svg" alt="CMake MaCOS Build Status"/> <br> 
           <a href="https://github.com/introlab/rtabmap/actions/workflows/cmake-ros.yml"><img src="https://github.com/introlab/rtabmap/actions/workflows/cmake-ros.yml/badge.svg" alt="CMake ROS Build Status"/> <br> 
           <a href="https://github.com/introlab/rtabmap/actions/workflows/docker.yml"><img src="https://github.com/introlab/rtabmap/actions/workflows/docker.yml/badge.svg" alt="Docker Build Status"/>
           </td>
        </tr>
     </tbody>
  </table>
 
 #### ROS Binaries
 
 `ros-$ROS_DISTRO-rtabmap`
 
 <table>
    <tbody>
        <tr>
           <td rowspan="1">ROS 1</td>
            <td>Noetic</td>
            <td><a href="http://build.ros.org/job/Nbin_ufv8_uFv8__rtabmap__ubuntu_focal_arm64__binary/"><img src="http://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__rtabmap__ubuntu_focal_arm64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td rowspan="4">ROS 2</td>
            <td>Humble</td>
            <td><a href="http://build.ros2.org/job/Hbin_uJ64__rtabmap__ubuntu_jammy_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__rtabmap__ubuntu_jammy_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Jazzy</td>
            <td><a href="http://build.ros2.org/job/Jbin_uN64__rtabmap__ubuntu_noble_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Jbin_uN64__rtabmap__ubuntu_noble_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Kilted</td>
            <td><a href="http://build.ros2.org/job/Kbin_uN64__rtabmap__ubuntu_noble_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Kbin_uN64__rtabmap__ubuntu_noble_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Rolling</td>
            <td><a href="http://build.ros2.org/job/Rbin_uJ64__rtabmap__ubuntu_jammy_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Rbin_uJ64__rtabmap__ubuntu_jammy_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
           <td>Docker</td>
           <td>
             <a href="https://hub.docker.com/r/introlab3it/rtabmap">rtabmap</a>
           </td>
           <td><img src="https://img.shields.io/docker/pulls/introlab3it/rtabmap" alt="Docker Pulls"/></td>
        </tr>
    </tbody>
</table>
