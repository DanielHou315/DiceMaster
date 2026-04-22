# Persona: Professor Alex Chen

## Who They Are

Professor Alex Chen is a 44-year-old university lecturer who teaches introductory Mandarin Chinese to undergraduate students at a mid-size public university. Alex holds a PhD in applied linguistics and has spent years studying how physical and kinesthetic activities improve vocabulary retention. Alex is not a programmer — their last exposure to code was a single Excel macro tutorial several years ago. They use Google Slides, Quizlet, and occasionally Kahoot for classroom activities.

Alex heard about DiceMaster at SeriousPlay 2025, where a short demo showed students shaking a device to cycle through vocabulary flashcards. The idea clicked immediately: this is the physical manipulation they've been trying to replicate with paper flashcards and dice-rolling activities, but with images and audio cues.

Alex's primary classroom goal with DiceMaster is a vocabulary quiz game: show a Chinese character on the top face, an English translation on the bottom face, and four image hints on the side faces. Students in groups of four sit around the device; one student shakes it to reveal a new question. The group discusses the answer before flipping it over.

**Technical baseline:** Minimal. Can edit a text file if given clear instructions. Has never used a terminal. Is willing to learn Python basics if the learning curve is bounded and there are copy-paste examples.

## Pain Points

- **Setup anxiety.** The phrase "install ROS2 and build from source" is a hard stop. Alex needs to know there is a path that does not require them to become a systems administrator to get a working device.
- **No Python experience.** Python is a prerequisite for writing DiceMaster games. Alex is willing to try but needs examples that look like fill-in-the-blank templates, not architectural explanations.
- **Asset preparation is unclear.** Alex has vocabulary lists in spreadsheets and images saved from language textbooks. The mapping from "images and a word list" to "a format DiceMaster understands" is not obvious without documentation.
- **Fear of breaking things.** Without understanding what the system is doing, Alex worries that a typo in a config file might brick the device or corrupt data. Clear error messages and rollback guidance matter.
- **Iteration speed.** If every change requires a rebuild or a restart, Alex will lose patience quickly. They need a tight loop: edit game file, run, see result on device within seconds.

## Journey

**Discovery at SeriousPlay 2025.** Alex watches a live demo and approaches the presenter with questions. Learns that the device is open-source and that game logic is written in Python. Takes a photo of the GitHub URL.

**First exploration.** Back in the office, Alex opens the DiceMaster repository. Reads the README. The ASCII architecture diagram and the quick example code snippet are reassuring — the example only has `start_strategy()`, a subscription, and a display call. Finds the Beginner's Game Development Guide link.

**Follows the beginner guide.** Works through the "Hello Dice" tutorial. Edits the provided template, changes the text strings to Chinese vocabulary words, adds a few JPEG images to the assets folder using the documented file structure. Runs the launch command. The device shows the vocabulary card.

**First real game.** Alex copies the `shake_quizlet` example game from the `examples/` directory as a starting point. Replaces the sample JSON vocabulary file with a list of 20 Chinese words and their English translations. Points `config.json` at a folder of copyright-cleared images (one per vocabulary word). The game runs in class the next week.

**Sharing with students.** Alex shows the game to students during a discussion section. Students take turns shaking the device. One student asks if they can add audio — Alex notes this as a feature request.

**Returning for more.** Semester later, Alex is back with a more ambitious idea: a grammar game where the dice shows a sentence with a blank on one face and four grammatical options on the side faces. Returns to the documentation, now reading the strategy API reference to understand how to handle the orientation events properly.

## Content Needs

- **Beginner game guide** with a true step-by-step walkthrough, including how to structure the asset folder, what JSON text files look like, and what the minimum viable `strategy.py` looks like
- **Annotated example games** that are short, heavily commented, and focused on a single concept each (shake only, orientation only, text only, images only)
- **Asset format reference** explaining exactly what file names and folder structure are required, with a concrete example using a vocabulary game
- **Error message glossary** explaining what the most common runtime errors mean and how to fix them, in plain language without ROS2 jargon
- **FAQ** covering "how do I add more questions," "how do I change fonts," and "how do I swap games without restarting"
