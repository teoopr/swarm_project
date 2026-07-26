# Sadržaj repozitorija

Repozitorij sadrži paket za višerobotski SLAM (`stage_multi_slam`) i paket za navigaciju flote u formaciji pomoću metode potencijalnih polja (`potential_fields`), a oboje se simulira u [*Stageu*](https://rtv.github.io/Stage/index.html).

Paketi su testirani u ROS 2 *[Humble Hawksbill](https://docs.ros.org/en/humble/index.html)* na Ubuntu 22.04 LTS.

Prije korištenja paketa, u workspace je potrebno klonirati i sljedeće repozitorije:

* `stage_ros2`: https://github.com/tuw-robotics/stage_ros2 - za simulaciju okruženja

* `m-explore-ros2`: https://github.com/robo-friends/m-explore-ros2 ([branch](https://github.com/robo-friends/m-explore-ros2/tree/feature/slam_toolbox_compat): `feature/slam_toolbox_compat`) - za spajanje mapa (`multirobot_map_merge`) i istraživanje granica (`explore_lite`)

Ovaj repozitorij je napravljen u sklopu rada: https://urn.nsk.hr/urn:nbn:hr:235:419354

# Priprema simulacije CRTA-e

Prije pokretanja bilo kojeg od paketa, potrebno je kopirati virtualno okruženje CRTA-e iz mape `world za Stage` u ovom repozitoriju u mape `stage_ros2/world` i `stage_ros2/world/bitmaps` nakon kloniranja *Stagea* u workspace.

Svaki robot ima svoj imenski prostor: `robot_0`, `robot_1`, `robot_2` i `robot_3`. Pri navigaciji flote, robot 0 je po zadanome predvodnik, a ostali su pratitelji.

# Višerobotski SLAM u *Stageu*

Paket `stage_multi_slam` pokreće potrebne podsustave za višerobotski SLAM kroz simulaciju u *Stageu*.

Sadržane su sljedeće *launch* datoteke:

* `four_robot_nav2.launch.py` - pokreće četiri instance *Navigation2*, svaka u imenskom prostoru određenog robota simuliranog pomoću *Stagea*

* `four_robot_slam_merge.launch.py` - pokreće četiri instance *SLAM Toolboxa* (također u imenskim prostorima) i jednu instancu čvora iz paketa `multirobot_map_merge`za spajanje 

* `four_robot_explore.launch.py` - pokreće čvorove za istraživanje granica iz paketa `explore_lite`

Sadržane su i konfiguracijske datoteke za svaki podsustav i za prikaz granica za istraživanje u RVizu. Prikaz granica se ne pokreće automatski nego je po želji potrebno ručno pokrenuti novu instancu RViza i učitati potrebnu konfiguracijsku datoteku.

## Primjer pokretanja višerobotskog SLAM-a

Svaku od naredbi pokrenuti u zasebnom terminalu.

1. Pokretanje simulacije u *Stageu* (terminal A):
   
   ```bash
   ros2 launch stage_ros2 stage.launch.py world:=crta_four_robots_slam
   ```

2. Pokretanje *SLAM Toolboxa* i spajanja lokalnih mapa (terminal B):
   
   ```bash
   ros2 launch stage_multi_slam four_robot_slam_merge.launch.py
   ```

3. Pokretanje *Navigation2* (terminal C):
   
   ```bash
   ros2 launch stage_multi_slam four_robot_nav2.launch.py
   ```
   
   Napomena: *Navigation2* se prvo pokreće za robote 0 i 1, zatim slijedi odgoda prije nego se pokrene i za robote 2 i 3.

4. Pokretanje istraživanja granica (terminal D):
   
   ```bash
   ros2 launch stage_multi_slam four_robot_explore.launch.py
   ```

Ako korisnik želi ručno navoditi robote umjesto korištenja istraživanja granica, to može pomoću paketa `teleop_twist_keyboard`, npr. za `robot_0`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard __ns:=/robot_0
```

Za prikaz granica za istraživanje pokrenuti RViz na sljedeći način npr. za `robot_0`:

```bash
ros2 run rviz2 rviz2 --ros-args -r /tf:=/robot_0/tf -r /tf_static:=/robot_0/tf_static
```

i učitati odgovarajuću pruženu konfiguracijsku datoteku za RViz.

## TO-DO za *stage_multi_slam*

- [ ]  koordinacija zadataka (tj. prostora) za mapiranje
- [ ]  možda zamijeniti `explore_lite` vlastitim paketom

# Navigacija flote u formaciji uz potencijalna polja

Paket `potential_fields` implementira [metodu potencijalnih polja](https://doi.org/10.1007/978-1-4757-1895-9_26) za navigaciju flote u tri formacije (romb, kvadrat, linija) i izbjegavanje prepreka. Omogućuje korištenje četiri načina lokalizacije: EKF ili UKF fuzija odometrije i IMU-a, s ili bez globalne korekcije poze kroz AMCL). 

Pruža i čvorove za evaluaciju točnosti lokalizacije i državanja formacije, te čvor za prikaz potencijalnih polja između dvije točke.

Pri prolazu kroz uski prolaz (prema definiranom parametru), formacija se mijenja u formaciju `linija` objavom na temu `/leader/uski_prostor`.

## Čvorovi paketa *potential_fields*

- `pf_controller.py` - glavni čvor za navigaciju u formaciji pomoću potencijalnih polja

- `noisy_odom_node.py` - dodaje šum u odometriju koju daje *Stage*

- `imu_sim_node.py` - pojednostavljeni simulator IMU-a koji za simulaciju mjerenja uzima stvarnu kutnu brzinu i dodaje šum

- `localization_evaluator.py` - čvor koji računa i zapisuje greške lokalizacije u zasebnu CSV datoteku

- `navigation_evaluator.py` - čvor koji računa i zapisuje greške održavanja formacije u zasebnu CSV datoteku

- `pf_prikaz_pub.py` - pri zadavanju cilja napravi prikaz potencijalnih polja između početne pozicije predvodnika (`robot_0`) i zadanog cilja navigacije za intuitivniju predodžbu gdje bi se flota kretala, ali ne uzima u obzir dinamičke prepreke (npr. članove flote) i ne ažurira prikaz periodički (samo ga stvori pri zadavanju cilja i objavljuje kao oblak točaka koji se može prikazati u RVizu)

## Konfiguracijske datoteke u paketu *potential_fields*

* `pf_parametry.yaml` - parametri za glavni controller i formaciju

* `pf.rviz` - konfiguracija za prikaz navigacije u RViz-u koja se automatski pokreće pri pokretanju glavne *launch* datoteke

* `pf_prikaz.rviz` - koristi se za prikaz potencijalnog polja kojeg daje čvor `pf_prikaz_pub.py`

## Primjer pokretanja navigacije flote

Paket sadrži jednu glavu *launch* datoteku: `pf_swarm.launch.py`

Paket se može pokrenuti na sljedeći način:

```bash
ros2 launch potential_fields pf_swarm.launch.py
```

i prima argumente prikazane i objašnjene u sljedećoj tablici.

| Argument         | Opis                                                                                                                                                                                              | Zadana vrijednost                   | Dopuštene vrijednosti                                                                     |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------- | ----------------------------------------------------------------------------------------- |
| `formacija`      | formacija flote                                                                                                                                                                                   | `romb`                              | `romb`, `kvadrat` ili `linija`                                                            |
| `use_sim_time`   | korištenje simulacijskog vremena                                                                                                                                                                  | `true`                              | `true` ili `false`                                                                        |
| `lokalizacija`   | odabir načina lokalizacije                                                                                                                                                                        | `ekf_amcl`                          | `ekf`, `ukf`, `ekf_amcl`, `ukf_amcl`                                                      |
| `sum_odometrije` | umjetno dodavanje šuma u odometriju koju daje *Stage*                                                                                                                                             | `true`                              | `true` ili `false`                                                                        |
| `lok_evaluacija` | uključuje evaluaciju točnosti lokalizacije                                                                                                                                                        | `true`                              | `true` ili `false`                                                                        |
| `nav_evaluacija` | uključuje evaluaciju točnosti održavanja formacije                                                                                                                                                | `false`                             | `true` ili `false`                                                                        |
| `lok_csv_path`   | putanja do CSV datoteke u koju se zapisuju greške lokalizacije                                                                                                                                    | `/tmp/pf_localization_error.csv`    | bilo koja valjana putanja koja završava s nazivom CSV datoteke (uključujući i ekstenziju) |
| `nav_csv_path`   | putanja do CSV datoteke u koju se zapisuju greške održavanja formacije (zadaje se za punu CSV datoteku, ali u istu mapu sprema i sažetu CSV datoteku kod koje dodaje `_summary` u naziv datoteke) | `/tmp/pf_navigation_evaluation.csv` | bilo koja valjana putanja koja završava s nazivom CSV datoteke (uključujući i ekstenziju) |
| `run_id`         | redni broj ponavljanja pokusa                                                                                                                                                                     | `0`                                 | cijeli broj                                                                               |
| `random_seed`    | seed za šum odometrije i IMU-a (ako je -1 onda ne određuje šum iz seeda)                                                                                                                          | `-1`                                | cijeli broj                                                                               |

Cilj se zadaje u automatski pokrenutom RVizu pomoću alata *2D Goal Pose* ili objavom poruke na temu `/goal_pose` kroz terminal (ovaj drugi način je koristan za ponavljanje istog cilja).

Primjer ručne objave cilja na temu:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \ "{header: {frame_id: 'map'}, pose: {position: {x: 7.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## TO-DO za *potential_fields*

- [ ]  dodatni globalni planer koji u obzir uzima i širinu formacije

- [ ]  robusnija lokalizacija članova flote kada nisu poznate početne poze robota

- [ ]  robusnije izbjegavanje drugih članova flote koje se ne oslanja samo na lidar
