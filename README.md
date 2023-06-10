# GroceryBee
GroceryBee is a robot to assist grocery stores in keeping up with their online orders. GroceryBee will help grocery store employees by collecting the items which are required for an online order from the store shelves, thus speeding up the delivery process. GroceryBee is designed to make decisions in an uncertain environment, so it is able to exploit real-time information from RGB-D cameras to make real-time changes in planning. GroceryBee is able to collect required items in response to the orders while navigating between the supermarket aisles without crashing. It can easily detect requested items on the shelves, picks them up safely and brings them to the delivery point for speedy delivery to customers. 

# Installation & Running the Project
1) ```git clone https://github.com/ardaOnal/grocery-bee```
2) Install python3, pip, venv.
```
sudo apt update
sudo apt install python3 python3-pip python3-virtualenv
```
3) Create a virtual environment in the project's root directory and install the requirements.
```
python3 -m venv env
source env/bin/activate
pip install -r requirements.txt
pip install torch torchvision
pip install -U git+https://github.com/luca-medeiros/lang-segment-anything.git

pip install python-tsp
sudo apt-get install python3-tk
```
  
4) To run the project, type ```python src/python/main.py```  
5) To view the output on Meshcat, go to ```http://localhost:7000``` on your browser (the URL may differ depending on which ports are currently in use on your PC so you can also check the console output to see which link to go to, for the Meshcat output).
6) After running the project, you can view the console output for information about various things happening while the project runs (for example the planning of the trajectory, segmentation, perception etc.).  
  
**Note:** You can switch to the ```multi-agent``` branch and run the project to view the simulation where two robots are working simultaneously to fulfil an order.

# Website
[Website](https://ardaonal.github.io/grocery-bee/)

# Poster
![grocerybee_poster](https://github.com/ardaOnal/grocery-bee/blob/main/Project%20Documents/GroceryBee_Poster_MidQ.png?raw_true)

# Demo Video
[![Watch the video](https://img.youtube.com/vi/afYTiJdF9Yk/maxresdefault.jpg)](https://youtu.be/afYTiJdF9Yk)
You can also checkout the [playlist](https://www.youtube.com/watch?v=GBnvu19R9jM&list=PL4TliZWXiW8y8ftY_JODTM1IjpGnXDufY&index=1&ab_channel=ArdaOnal) with all of our demo videos.  

# Members
![alt text](https://github.com/ardaOnal/grocery-bee/blob/main/GroceryBee.png?raw=true)
From left to right: Arda Önal, Emir Melih Erdem, Mert Barkın Er, Efe Beydoğan, Eren Polat

- [Arda Önal](https://www.linkedin.com/in/ardaonal/) 21903350
- [Efe Beydoğan](https://www.linkedin.com/in/efebeydogan/) 21901548
- [Emir Melih Erdem](https://www.linkedin.com/in/emir-melih-erdem/) 21903100
- [Eren Polat](https://www.linkedin.com/in/eren-polat323/) 21902615
- [Mert Barkın Er](https://www.linkedin.com/in/mertbarkın/) 21901645
