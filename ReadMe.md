## Installation

To install the Emotional Offline Voice Assistant, follow these steps:

1. Clone the repository:

```bash
   get the Microsoft Visual Studio
   python -m venv venv
```
   if strugles of creating venv theen execute this code:

```bash
   Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser -Force
   python -m venv venv
   # or
   C:\Users\vboxuser\AppData\Local\Programs\Python\Python311\python.exe -m venv venv

```

Activate the venv

```bash
   ./venv/Script/activate
   # or
   .\venv\Scripts\activate
   # or
   ./venv/Scripts/Activate.ps1
```

   Now clone this repository
```bash
   git clone https://github.com/Curiosity-AI-team/Reinfocement-Learing-Projects.git
```


Install torch with cuda
https://pytorch.org/get-started/locally/
To test that the cuda is working:
```bash
# activate your venv first
cd Reinfocement-Learing-Projects
python docs/cuda_test.py
```
```bash
   cd Reinfocement-Learing-Projects
   pip install -r requirements.txt