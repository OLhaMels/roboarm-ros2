from flask import Flask, render_template_string, request
from RpiMotorLib import RpiMotorLib


# ============================================================
# GPIO CONFIGURATION
# BCM GPIO numbering
# ============================================================

# A4988 microstepping pins
# MS1 -> GPIO17
# MS2 -> GPIO27
# MS3 -> GPIO22
GPIO_PINS = (17, 27, 22)

# A4988 control pins
DIRECTION_PIN = 20
STEP_PIN = 21


# ============================================================
# STEPPER MOTOR
# ============================================================

motor = RpiMotorLib.A4988Nema(
    DIRECTION_PIN,
    STEP_PIN,
    GPIO_PINS,
    "A4988"
)


# ============================================================
# FLASK
# ============================================================

app = Flask(__name__)


HTML = """
<!DOCTYPE html>
<html>

<head>
    <title>Raspberry Pi 5 Stepper Control</title>

    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            margin-top: 50px;
        }

        .container {
            width: 500px;
            margin: auto;
        }

        input[type="range"] {
            width: 400px;
        }

        button {
            margin-top: 20px;
            padding: 10px 30px;
            font-size: 18px;
        }

        #sliderValue {
            font-size: 24px;
            font-weight: bold;
        }
    </style>
</head>

<body>

<div class="container">

    <h2>Raspberry Pi 5 Stepper Motor Control</h2>

    <p>
        Counter-clockwise &lt; 10
        |
        STOP = 10
        |
        Clockwise &gt; 10
    </p>

    <form method="POST" action="/motor">

        <input
            type="range"
            min="1"
            max="20"
            value="10"
            name="slider"
            id="slider"
            oninput="sliderValue.innerText = this.value"
        >

        <p>
            Value:
            <span id="sliderValue">10</span>
        </p>

        <button type="submit">
            Move Motor
        </button>

    </form>

</div>

</body>
</html>
"""


# ============================================================
# HOME PAGE
# ============================================================

@app.route("/")
def home():
    return render_template_string(HTML)


# ============================================================
# MOTOR CONTROL
# ============================================================

@app.route("/motor", methods=["POST"])
def motor_control():

    try:
        slider = int(request.form["slider"])
    except (KeyError, ValueError):
        return "Invalid slider value", 400

    print(f"Slider value: {slider}")

    # --------------------------------------------------------
    # STOP
    # --------------------------------------------------------

    if slider == 10:
        print("Motor stopped")
        return render_template_string(HTML)

    # --------------------------------------------------------
    # MOTOR SPEED
    #
    # Slider:
    #
    # 1  = fastest counter-clockwise
    # 9  = slow counter-clockwise
    # 10 = stop
    # 11 = slow clockwise
    # 20 = fastest clockwise
    # --------------------------------------------------------

    speed_level = abs(slider - 10)

    # Delay between STEP pulses
    # Larger delay = slower motor
    # Smaller delay = faster motor
    step_delay = 0.006 - (speed_level * 0.0005)

    # Safety minimum
    if step_delay < 0.001:
        step_delay = 0.001

    # Number of motor steps per command
    steps = 600

    # --------------------------------------------------------
    # CLOCKWISE
    # --------------------------------------------------------

    if slider > 10:

        print(
            f"Clockwise | "
            f"steps={steps} | "
            f"delay={step_delay:.4f}s"
        )

        motor.motor_go(
            True,           # Clockwise
            "Full",         # Full-step mode
            steps,          # Number of steps
            step_delay,     # Step delay
            False,          # Verbose output
            0.05            # Initial delay
        )

    # --------------------------------------------------------
    # COUNTER-CLOCKWISE
    # --------------------------------------------------------

    elif slider < 10:

        print(
            f"Counter-clockwise | "
            f"steps={steps} | "
            f"delay={step_delay:.4f}s"
        )

        motor.motor_go(
            False,          # Counter-clockwise
            "Full",         # Full-step mode
            steps,          # Number of steps
            step_delay,     # Step delay
            False,          # Verbose output
            0.05            # Initial delay
        )

    return render_template_string(HTML)


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    print("-----------------------------------")
    print(" Raspberry Pi 5 Stepper Controller")
    print("-----------------------------------")
    print()
    print("Open in browser:")
    print("http://<RASPBERRY_PI_IP>:5000")
    print()

    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        threaded=False
    )