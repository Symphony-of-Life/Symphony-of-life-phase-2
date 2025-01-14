# GLOW Fmod 
This is the repo for the glow FMOD project.
Both to build and to edit the fmod project, you'll need [Fmod Studio](https://www.fmod.com/download).
## Using fmod in production
To use FMOD in production. Build the project.
Go to Fmod Studio, go to File -> Build all platforms. 
You'll need to figure out which settings and confugurations work for you.

## Understanding FMOD
The project has three tabs, Events are the core of the program. Here variables and sound effects are managed and connected to parameters.
Banks control output volume and which cannels are activated.
Assets are all the individual song assets on all the machines.
<img width="1728" alt="Scherm­afbeelding 2024-09-19 om 10 57 59" src="https://github.com/user-attachments/assets/66a87977-7a19-45fe-bf5f-5ef21df14594">
In a event tab music page, you can find the variables on top of the page. Changing the activation value (0.0000 to 1.0000) changes the volume of the audiotrack. At the bottom of the page you can see which surround sound speaker this track sends sounds to.
<img width="1728" alt="Scherm­afbeelding 2024-09-19 om 10 58 59" src="https://github.com/user-attachments/assets/8865d87b-5d59-4b24-ba82-75a51fc43bac">
The songs are stored in assets. Here you can find all previous songs and sound effects as well.
<img width="299" alt="Scherm­afbeelding 2024-09-19 om 10 59 59" src="https://github.com/user-attachments/assets/cb9cf3ad-d76e-4562-b341-06206ba1a103">
