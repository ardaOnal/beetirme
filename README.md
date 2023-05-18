# GroceryBee
GroceryBee is a robot to assist grocery stores in keeping up with their online orders. GroceryBee will help grocery store employees by collecting the items which are required for an online order from the store shelves, thus speeding up the delivery process. GroceryBee is designed to make decisions in an uncertain environment, so it is able to exploit real-time information from RGB-D cameras to make real-time changes in planning. GroceryBee is able to collect required items in response to the orders while navigating between the supermarket aisles without crashing. It can easily detect requested items on the shelves, picks them up safely and brings them to the delivery point for speedy delivery to customers. 

# Members
- [Arda Önal](https://www.linkedin.com/in/ardaonal/) 21903350
- [Efe Beydoğan](https://www.linkedin.com/in/efebeydogan/) 21901548
- [Emir Melih Erdem](https://www.linkedin.com/in/emir-melih-erdem/) 21903100
- [Eren Polat](https://www.linkedin.com/in/eren-polat323/) 21902615
- [Mert Barkın Er](https://www.linkedin.com/in/mertbarkın/) 21901645

# Installation
1) ```git clone https://github.com/ardaOnal/grocery-bee```
2) Install python3, pip, venv.
```
sudo apt update
sudo apt install python3 python3-pip python3-virtualenv
```
4) Create a virtual environment in the project's root directory and install the requirements.
```
python3 -m venv env
source env/bin/activate
pip install -r requirements.txt
pip install torch torchvision
pip install -U git+https://github.com/luca-medeiros/lang-segment-anything.git
```
