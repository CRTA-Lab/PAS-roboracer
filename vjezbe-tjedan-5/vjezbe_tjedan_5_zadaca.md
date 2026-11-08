# Zadaća – Vježbe, tjedan 5
## RoboRacer simulator i razvojno okruženje

**Cilj:** Samostalno pokrenuti RoboRacer simulator, istražiti osnovne ROS alate, Docker i Git.

---

## 1. Preuzmi i pokreni simulator

Za početak je potrebno preuzeti repozitorij s GitHuba na vlastito računalo.

U repozitoriju se nalazi mapa `/vjezbe-primjeri`, unutar koje se nalaze upute za pokretanje simulatora.

Na GitHub Container Registryju (GHCR) dostupan je Docker image koji sadrži RoboRacer simulator i primjer implementacije algoritma `wall_follow`.

---

## 2. Osnove Docker-a

Nakon uspješnog pokretanja simulatora trebala bi biti aktivna dva Docker containera.

### Zadaci

1. Istražite Docker command-line naredbe. Kako se zovu ti containeri? Iz kojih Docker imageova su napravljeni?

2. Napravite novi folder u `/vjezbe-primjeri` na vlastitom računalu, naziva `student_2`. Kako biste taj folder preslikali u Docker container koji sadrži simulator?

3. Kreirajte novi Docker container nazvan svojim imenom i prezimenom, koristeći isti Docker image kao i container u kojem je pokrenut simulator. Uđite u novokreirani container i izvršite naredbu `echo $ROS_DOMAIN_ID`. Izvršite istu naredbu na vlastitom računalu. Kako možete osigurati da vrijednosti varijable `ROS_DOMAIN_ID` budu jednake unutar Docker containera i na vašem računalu?

---

## 3. Osnove ROS-a

Folder `/vjezbe-primjeri/student` razvojno je okruženje za testiranje vlastitog koda.

### Zadaci

1. Kreirajte ROS 2 paket proizvoljnog naziva unutar mape `/vjezbe-primjeri/student`.

2. Unutar kreiranog paketa implementirajte ROS 2 čvor imena `scan_processor` koji će se pretplatiti na temu `/scan`.

3. Čvor treba obrađivati pristigle poruke i na temu `/closest_measurement` objavljivati trenutno najbliže mjerenje s teme `/scan`.

4. Koji se tip poruke objavljuje na temi `/closest_measurement`?

### Parametrizacija čvora

Proširite implementirani čvor dodavanjem ROS 2 parametra `measurement_type`, koji određuje koje će se mjerenje objavljivati na temi `/closest_measurement`.

Parametar treba podržavati dvije vrijednosti:

- `closest` – objavljuje najbliže mjerenje.
- `farthest` – objavljuje najdalje mjerenje.

Provjerite ispravnost rada čvora za obje vrijednosti parametra. Parametar mora biti vidljiv naredbom:

```bash
ros2 param dump /scan_processor
```

---

## 4. Osnove Git-a

### Zadaci

1. Kreirajte novu Git granu naziva test-branch. Koja je svrha korištenja grana u Git-u?

2. Na novoj grani napravite mapu proizvoljnog naziva i unutar njega .txt datoteku proizvoljnog naziva. Unutar .txt datoteke napišite svoje ime i prezime. Spremite promjene s pripadajućom proizvoljnom porukom.

3. Ponovno otvorite tekstualnu datoteku i u novi red napišite PAS-roboracer. Zatvorite tekstualnu datoteku. Kojom naredbom možete vidjeti izmjene?

4. Ponovno spremite promjene s pripadajućom proizvoljnom porukom. Kojom naredbom možete vidjeti sve prijašnje promjene?

5. Od dvije zasebno spremljene promjene napravite jednu veliku. Zadržite poruku zadnje promjene.

6. Vratite se na glavnu granu i izbrišite test-branch.