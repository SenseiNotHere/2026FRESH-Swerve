# 🚀 1811 2026 Swerve  

## 🏆 Description  
This project is a **modification** of the [CommandRevSwerve](https://github.com/epanov1602/CommandRevSwerve) repository. Instead of using the **REV library for Spark** motor controllers, this version is adapted to use the **Phoenix6 library for TalonFX** motor controllers.  

🛠️ **Developed by:** **FRC Team 1811**  

---

## ✨ Features  
✔️ Written in **Python** for **FRC teams**  
✔️ Uses **TalonFX** motor controllers with the **Phoenix6** library  
✔️ Provides a **base swerve drive system** for teams using TalonFXs  

---

## 🔧 Installation  
We recommend using **PyCharm** 🖥️ as the IDE for this project. Follow these steps to set up the environment:  

1️⃣ Open **PyCharm** and clone the project.  

2️⃣ At the bottom of the screen, click on **`<No Interpreter>`**, then:  
   - Select **"Add New Interpreter"**  
   - Choose **"Add Local Interpreter"**  
   - Click **✅ OK** to confirm.

3️⃣ Open a **terminal** in the project directory and install dependencies:  
   ```sh
   pip install -r requirements.txt
   ```

4️⃣ Use the Sync configuration to install other needed things.

🎮 Usage

To use this code, you should have a swerve-drive robot 🤖 with motors that use TalonFX motor controllers.

⚡ Steps:
✅ Verify your hardware: Ensure your robot setup is correct. We are using KrakenX60s for our swerve drive.
🛠️ Modify constants.py:
- Check and update motor CAN IDs, sensor settings, and other parameters based on your robot’s configuration.
- ⚠️ Double-check all values before deploying!
🚀 Deploy the code:
Use the Deploy configuration to deploy the code to your robot.
## ⚠️ Be extremely careful, as issues most likely will arise during initial testing.
🤝 Contributing

This project is primarily maintained by FRC Team 1811. If you have suggestions or improvements, feel free to fork the repository and submit a pull request! 🚀

📜 License

This project follows the same licensing as the original CommandRevSwerve repository.

🔥 Made with ❤️ by FRC Team 1811

## THIS CODE HAS ONLY BEEN TESTED IN A SIMULATOR! USE IT AT YOUR OWN RISK!
