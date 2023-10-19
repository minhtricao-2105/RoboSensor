# MARKETMATE FETCHER
## Back ground:
In nowadays trend, the integration of collaborative robots (cobots) is reshaping the dynamics of retail operations. This project explores the innovative application of cobots in the supermarket domain, focusing on their ability to streamline product restocking. By using robot's application which seeks to revolutionize the supermarket experience, optimizing efficiency and enhancing customer satisfaction in the realm of automated retail solutions.

In the project, we decided to use Omron robots which are TM5 (with mobile base) and TM12 for restocking process.
The TM5 will be on top of mobile base which increase the accessibity and flexibity, TM5 also has compact design which make it ideal for filling products in small shelves
TM12 has larger workspace so it can reach items from greater distance and it has better load distribution.

Our control mechanism relies primarily on the robust RMRC (Resolved Motion Rate Control) algorithm, ensuring seamless and precise robot navigation between designated positions within the workspace. Emphasizing user safety, our system incorporates a range of protective measures, including simulated and physical emergency-stop (e-stop) buttons, strategically positioned light curtains, and advanced sonar sensors for collision detection in the workspace. Additionally, a fail-safe collision avoidance feature has been integrated combining with algorithm to check self collision and ground collision, ensuring continued safety even in the event of sensor malfunction, thereby prioritizing a secure operational environment.

Simulated within the Matlab environment, our project envisions a dynamic workspace inspired by the layout of a contemporary convenience store. The process involves the TM12, facilitating the transfer of products to a designated location. Subsequently, the TM5 mobile unit approaches the same point, efficiently picking and storing the items within its container. Following this, the TM5 navigates each shelf, ensuring product placement in their respective locations. During process, system can be able to recover/resume after an e-stop event. 

We also created the GUI for user to interact with the system. The GUI has advanced “teach” functionality that allows jogging the robot. It includes both individual joint movements  plus enable [x,y,z] Cartesian movements. A joystick is can be used as an additional control method beside GUI buttons presses.

In real robot demonstration, ...
