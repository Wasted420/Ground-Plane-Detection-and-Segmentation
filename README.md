# Ground Plane Detection and Segmentation (FPGA/Verilog)

Acest proiect implementează o soluție hardware în **Verilog** pentru segmentarea în timp real a planului solului (Ground Plane Segmentation) din nori de puncte LiDAR. Sistemul este proiectat pentru aplicații de **navigație autonomă și detecția obstacolelor**, utilizând o abordare eficientă bazată pe canale (channel-based).

## Descriere Proiect

Obiectivul principal este separarea punctelor care aparțin solului de cele care reprezintă obstacole. Deși algoritmi precum RANSAC sunt standardul în industrie pentru detectarea planelor, aceștia sunt computațional intenși și dificil de paralelizat eficient pe hardware limitat.

Acest proiect adoptă o abordare **Stream-Based / Channel-Based**, optimizată pentru FPGA, care procesează datele secvențial pe măsură ce sunt primite de la senzor.

### Metodologie Implementată
Conform sugestiilor din literatura de specialitate, proiectul implementează:
1.  **Abordare Channel-based:** O alternativă mai rapidă la RANSAC, ideală pentru procesarea în flux (stream processing).
2.  **Filtrare Savitzky-Golay:** Netezirea datelor de adâncime (Range Smoothing) pentru eliminarea zgomotului inerent senzorilor LiDAR, păstrând în același timp caracteristicile importante ale terenului.
3.  **Segmentare bazată pe pantă:** Clasificarea punctelor pe baza gradientului geometric dintre eșantioane consecutive.

## Structura Repository-ului

* `lidar_segmentation.v`: Codul sursă Verilog principal. Include:
    * **Window Buffer:** Registru de deplasare pentru analiza ferestrei glisante (5 puncte).
    * **Savitzky-Golay Filter:** Modul DSP pentru netezirea semnalului.
    * **Segmentation Logic:** Blocul de decizie (Sol vs. Obstacol).
* `tb_lidar.v`: Testbench complet pentru simulare. Generează scenarii de test (teren plat, rampe, obstacole bruște).
* `rezultate.vcd`: Fișier de ieșire (Waveform) generat în urma simulării.

## Detalii Tehnice și Algoritm

### 1. Filtrare Savitzky-Golay
Pentru a curăța datele brute de intrare ($Z_{raw}$), se utilizează un filtru polinomial de netezire cu 5 coeficienți, optimizat pentru operații pe biți (fără împărțiri complexe):

$$Y = \frac{17p_2 + 12p_3 + 12p_1 - 3p_4 - 3p_0}{32}$$

*Notă: Împărțirea la 32 este realizată hardware printr-o shiftare la dreapta (`>>> 5`).*

### 2. Logica de Segmentare
Decizia de clasificare se bazează pe panta dintre punctul curent și cel anterior. Dacă variația de înălțime ($\Delta Z$) este prea mare raportată la distanța parcursă ($\Delta R$), punctul este marcat ca obstacol.

```verilog
// Pseudocod logică segmentare
if (delta_z < (delta_r * SLOPE_THRESHOLD))
    is_ground = 1; // Teren navigabil
else
    is_ground = 0; // Obstacol (Zid, Bordură, etc.)
```

### 3. Instrucțiuni de Utilizare
Proiectul poate fi simulat folosind Icarus Verilog și vizualizat cu GTKWave.

Pasul 1: Compilare
Deschide terminalul în folderul proiectului și rulează comanda:
iverilog -o simulare.out tb_lidar.v lidar_segmentation.v

Pasul 2: Rulare Simulare
Rulează executabilul generat pentru a efectua simularea:
vvp simulare.out
Vor fi afișate în consolă etapele testării (ex: "Testare SOL PLAT...", "Testare OBSTACOL...") definite în testbench.

Pasul 3: Vizualizare Grafică
Deschide formele de undă rezultate pentru analiză:
gtkwave rezultate.vcd
În GTKWave, adaugă semnalele raw_z_in, filtered_z și segmentation_result pentru a observa detecția obstacolelor.

## 📚 Bibliografie și Referințe

Acest proiect a fost dezvoltat pe baza următoarelor lucrări de cercetare și documentații tehnice:

* **FPGA Optimization:**
  * Zhang, Xiao, et al. "Stream-Based Ground Segmentation for Real-Time LiDAR Point Cloud Processing on FPGA." *arXiv preprint arXiv:2408.10410* (2024).

* **Algorithm Reference:**
  * MathWorks. "Ground Plane Segmentation of Lidar Data on FPGA". *Technical Documentation* (2024). [Link](https://www.mathworks.com/help/visionhdl/ug/lidar-ground-segmentation.html)

* **Adaptive Methods:**
  * Vu, Hoang, et al. "Adaptive ground segmentation method for real-time mobile robot control." *International Journal of Advanced Robotic Systems* 14.6 (2017).

* **Comparative Survey (RANSAC):**
  * Martínez-Otzeta, José María, et al. "Ransac for robotic applications: A survey." *Sensors* 23.1 (2022): 327.
  * Zeineldin, Ramy Ashraf, and Nawal Ahmed El-Fishawy. "A survey of RANSAC enhancements for plane detection in 3D point clouds." *Menoufia J. Electron. Eng. Res* 26.2 (2017).
