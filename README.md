# Speaking-Hands

The initial idea behind this device was to bridge the communication gap between mute and blind people. The glove helps capture sign language and converts it to speech so that the other person can understand the hand gestures without knowing the standard signs. This idea was then extended to not just any standard sign languages such as ASL(American Sign Language) or BSL(British Sign Language) but for people to come up with their very own sign language, thereby easing the communication further.


![speaking_hands](https://github.com/sag-27/Speaking-Hands/assets/117821445/21e35d55-c939-4c2a-b8bd-4db9850f3071)

The glove is fitted with micro potentiometers on every finger that stretch tas the finger bends, producing a change in the resistance values. The palm of the glove also has an attached gyroscope module to identify the orientation of the palm, producing different yaw, pitch, roll values. The combination of these five resistance values and three gyroscope readings remain unique for every gesture, thereby making it the key to identify its corresponding sudio signal. Various Machine Learning Algorithms such as Naïve Bayes, SVM, MLP Neural Networks were implemented to train the data.
