import speech_recognition as sr
import pyttsx3

# Initialize the text-to-speech engine once
engine = pyttsx3.init()

# Function to convert text to speech
def SpeakText(command):
    engine.say(command)
    engine.runAndWait()

# Map commands to actions
def map_command_to_action(command):
    actions = {
        "start adam": "Car powering on",
        "turn left": "Car turning left",
        "turn right": "Car turning right",
        "go forward": "Car moving forward",
        "adam stop": "Car stopping",
    }
    return actions.get(command, None)

# Continuous listening for commands
def main():
    print("Voice-controlled car is ready. Say 'start adam' to begin.")
    SpeakText("Voice-controlled car is ready. Say 'start adam' to begin.")
    car_running = False  # To track if the car is active

    recognizer = sr.Recognizer()  # Initialize the recognizer once

    while True:
        try:
            with sr.Microphone() as source:
                print("Listening for your command...")
                recognizer.adjust_for_ambient_noise(source, duration=0.2)
                audio = recognizer.listen(source)

                # Recognize the command
                command = recognizer.recognize_google(audio).lower()
                print(f"Did you say: {command}")
                SpeakText(f"Did you say: {command}?")

                # Map the command to an action
                action_response = map_command_to_action(command)
                if action_response:
                    print(action_response)
                    SpeakText(action_response)

                    if command == "start adam":
                        car_running = True

                    elif command == "adam stop":
                        if car_running:
                            print("Car is stopping. Exiting the program.")
                            SpeakText("Car is stopping. Exiting the program.")
                            break
                        else:
                            print("Car is not running.")
                            SpeakText("Car is not running.")
                else:
                    print("Command not recognized.")
                    SpeakText("Command not recognized.")

        except sr.UnknownValueError:
            print("Sorry, I could not understand the audio.")
            SpeakText("Sorry, I could not understand the audio.")
        except Exception as e:
            print(f"An error occurred: {e}")
            SpeakText("An error occurred. Please try again.")

if __name__ == "__main__":
    main()
