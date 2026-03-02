# 3D Printed Single-Wire Sensing 

About
-----
* Source code of a computational design pipeline to generate a 3D Printed Object with capacitive touchpoints using single-wire sensing
  - The pipeline is introduced in "Computational Design and Single-Wire Sensing of 3D Printed Objects with Integrated Capacitive Touchpoints", S. Sandra Bae, Takanori Fujiwara, Ellen Yi-Luen Do, Danielle Albers Szafir, and Michael L. Rivera. In Proc. SCF, 2025 (forthcoming).
  - This pipeline is extended from the one in **["A Computational Design Pipeline to Fabricate Sensing Network Physicalizations"](https://arxiv.org/abs/2308.04714)**, S. Sandra Bae, Takanori Fujiwara, Anders Ynnerman, Ellen Yi-Luen Do, Michael L. Rivera, and Danielle Albers Szafir. IEEE Transactions on Visualization and Computer Graphics, vol. 30, no. 1, pp. 913-923, Jan. 2024. [Related source code.](https://github.com/takanori-fujiwara/sensing-network)

* Links:
  * [Demo video](https://youtu.be/at5puV1MHUo?si=asOLS72rP1dc6IvJ)
  * [arXiv paper (coming soon)](#)

******

Content
-----
* `interface_design`: Web-based UI to select touchpoints and wiring connection points.
* `sw_sensing`: Python library for automatic circuit design of single-wire (or double-wire) sensing for freeform interfaces.


******

1.Interface Design Web UI (`interface_design`)
-----

### Installation requirements
* Node.js (latest)
* Browser supporting JavaScript ES2015(ES6) and WebGL 2.0

* Note: Tested with macOS Sequoia with Google Chrome.

### Setup

* Move to `interface_design` directory; then run the below command in Terminal:

    `npm install`

## Usage

* Run

    `npx vite`

* With your browser, open the URL indicated by the above command.

* To read a different STL file, change the blow line in `script/main.js`:

    `const stlFilePath = './models/bunny.stl';`

    * Note: To avoid cross reference errors, for now, `interface_design` has its own `models` directory (i.e., this is different from `models` located at the top directory)

******

2. Python library for automatic single-wire sensing circuit design (`sw-sensing` library)
-----

### Installation requirements
* Python3.12
* Note: Currently, only supports Python3.12 due to the required library dependencies. 
    - The main challenge to support Python3.13 is VTK's slight behavior difference after thair major update, which makes it difficult to debug the code.
* Note: Tested with macOS Sequoia.

### Setup

* Install `graph-tool` for yoru Python3.12 environment. Follow `graph-tool`'s [install instruction](https://git.skewed.de/count0/graph-tool/-/wikis/installation-instructions).

  * For macOS, a reasonable option is using anaconda's virtual environment. For example, after installing anaconda:

    `conda create --name sws -c conda-forge graph-tool python=3.12`
    
    `conda activate sws`

* While the above environment set up with graph-tool is active, run the below command after moving to this repository:

    `pip3 install .`


## Usage

* See examples provided in `examples` directory. Detailed documentations will be provided in the near future.

    - For example, you can run: `cd examples` then `python3 land_single.py`. This code will generate three stl files, land.resistor.stl, land.link.stl, land.node.stl, in `model` directory. 


******

3. Preparation for fabrication
-----

* To fabricate a freeform interface, you need to have four stl files made with the above `sw-sensing` libarary.

  a. Original 3D model (interface exterior)
  b. Conduits (cases of conductive traces. By default, named *.link.stl)
  c. Conductive traces (traces for resistors. By default, named *.resistor.stl)
  d. Touchpoints and wiring connection points (By default, named *.node.stl)

* Print a 3D object with the above four stl files using conductive and non-conductive materials. For example, you can use PrusaSlicer for this preparation.

  a. Original 3D model: non-conductive materials, infill density at least 5 % (e.g., 20%)
  b. Conduits: non-conductive materials, high infill density (e.g., 100%)
  c. Conductive traces: conductive materials, high infill density (e.g., 100%)
  d. Touchpoints and wiring connection points: conductive materials, high infill density (e.g., 100%)

* Build an electric circuit with Arduino Uno R4. Refer to `sensing-network` repository and use source code in their `arduino` directory: [https://github.com/takanori-fujiwara/sensing-network](https://github.com/takanori-fujiwara/sensing-network).

* Perform calibration. Refer to `sensing-network` repository and use source coe in their `callibration` directory: [https://github.com/takanori-fujiwara/sensing-network](https://github.com/takanori-fujiwara/sensing-network).


******
License
-----

See License.txt (Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License. Copyright: Takanori Fujiwara and S. Sandra Bae.)

******
How to cite
-----

S. Sandra Bae*, Takanori Fujiwara*, Danielle Szafir, Ellen Yi-Luen Do and Michael L. Rivera (*equally contributed), "Computational Design and Single-Wire Sensing of 3D Printed Objects with Integrated Capacitive Touchpoints." In Proc. SCF, 2025 (forthcoming).


