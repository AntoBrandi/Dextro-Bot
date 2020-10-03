from __future__ import print_function
import os.path
import os
import playsound
import speech_recognition as sr
from gtts import gTTS
from ros_interface import RosInterface

# If modifying these scopes, delete the file token.pickle.
SCOPES = ['https://www.googleapis.com/auth/calendar.readonly']
WAKE_STRS = "ok jarvis"
THANK_STRS = "thank you"


# Function that reads a string of text to the user
def speak(text):
    tts = gTTS(text=text, lang="en")
    filename = "voice.mp3"
    tts.save(filename)
    playsound.playsound(filename)
    os.remove("voice.mp3")


# Function that converts the speech of an user to a text using the google speech to text api
def get_audio():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        audio = r.listen(source)
        said = ""

        try:
            # use the google api in order to convert the speech to text
            said = r.recognize_google(audio)
            print(said)
        except Exception as e:
            # in case of error during the listening phase of the user voice
            print("Exception: " + str(e))

    return said.lower()


# ros = RosInterface('192.168.0.166')


while True:
    # convert the speech to text
    text = get_audio()

    # detect if the user said something to trigger the assistant
    if text.count(WAKE_STRS) > 0:
        # The assistant is triggered and is ready to answer questions or doing stuffs
        speak("Hi, how can I help?")
        # Wait for the user to provide a task to the assistant
        text = get_audio()

        # Detect the type of task the user gave to teh assistant and complete it
        if "hello" in text:
            speak("Hello, how are you?")

        if "your name" in text:
            speak("My name is Jarvis")

        if "how are you" in text:
            speak("I am a computer, of course I'm fine")

        if "follow" in text:
            speak("OK, let's go")
            # create an instance of the ros interface class that will publish messages on ROS topics
            # ros.publish('/jarvis')

        if "free" in text:
            speak("OK, see you later")
            # ros.publish('/jarvis')

        if "stop" in text:
            speak("OK, see you later")
            # ros.publish('/jarvis')

    # detect if the user thank the assistant
    if text.count(THANK_STRS) > 0:
        speak("You are welcome")
