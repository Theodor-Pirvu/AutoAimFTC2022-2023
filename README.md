## About
Using EasyOpenCV and the Java programming language, I have created a feauture that will help our bot during the FTC challenge score more points by auto correcting the its wrist to the nearest pole. By developing a good processing frame and using mathematics to create it's own pixel coordinate system, I was able to turn locations of the pole, and find the angle between the bot and the nearest pole, and later convert them to degree's of rotation for its servo. Overall it was a good challenge, and it really helped us a lot! To reach the file, TeamCode > src/main > java/org/firstinspires/ftc/teamcode > OkHearMeOut (file name).

## How It Works
With the coordinate system set up, using EasyOpenCV's built in frame processing, and a little bit of math sprinkled on top, it was returning the pole x-midpoint, the pole area and other aspects that I wasn't using. By using inverse tangents, I was able to get the angle of the bot's direction with the pole's midpoint. With this inverse tangent angle, I was able to convert it to a servo angle with some more math. Now with the pole area, I set up a graph to find an average finding area equation, that takes in the pole's area, and turns it into inch parameters. I used this to automatically get our bot's swivel closer toward the pole. With these two parameters (the angle of the pole and the distance between the pole and the bot), with the click of a button or automatically, the precision that our bot was able to get was unmatched compared to other bots during the competition, which allowed us to win the State Championship and get qualified to the worlds competition in Housten, Texas. 

Overall I really enjoyed this project because it brought me closer to my dream, and I am glad I was able to complete it with the time cruch I had on my hands. (Also shout out to David for helping me out.)

Milestone images:

![autoaim](https://user-images.githubusercontent.com/110932395/221076253-981221e3-ba5e-46de-8fb3-60c0c92acedc.jpg)


![autoaimmath2](https://user-images.githubusercontent.com/110932395/221076340-dce8ff40-db9d-427b-ad16-d0201fb9bad4.jpg)


![auto_aim_math](https://user-images.githubusercontent.com/110932395/221076532-d5c98725-39c2-43fc-bcd0-a692e06c8709.jpg)

