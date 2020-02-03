# Maki Development 

**Discovering the Neural Mechanisms of Dyadic Social Interaction**

-----------

Maki is a programmatically emotive robot, currently (as of 12-Nov-2019) implemented with the following behaviors. 

**[Default]** Upon startup, Maki assumes an ‘awake’ position, looking ahead with eyes fully open and head in parallel with the neck. Maki spontaneously blinks approximately every 1 to 2.5 seconds. *Action*: button press, `“roslaunch maki_robot test_some_behaviors.launch”`

**[Reset]** Maki attempts to reset to the default behavior. *Action*: `“neutral”` command.

**[Nothing]** Maki pauses all behaviors including spontaneously blinking. *Action*: `"nothing"` command.

**[Looking]** Maki looks up and at experimenter (which will always be on the right). *Action* `upright` or `right` commands. 

**[Thinking]** To demonstrate thinking through lateral eye movements, Maki engages in visual scanning, modelled after the behavior in infants. There are currently two variations of transitioning into this thinking behavior, turning and looking either far left or slightly left. *Actions*: `“think”` and `“wonder”` commands, respectively.

**[Responding]** Responding to participant prompts are expressed as nodding the head ‘yes’ and shaking the head ‘no’. There are currently 3 variants for each response. Visual scanning to express thinking is not implemented for these responses. *Actions*: `“nod full”`, `“nod up”`, `“nod down”`, and `“shake full”`, `“shake right”`, `“shake left”` commands.

Executing behaviors involves first launching an `roscore` instance. Then, in a separate terminal, running `roslaunch maki_robot test_some_behaviors.launch`. In the same terminal, the previously listed behaviors can be individually executed by sending the respective commands.

---------
**Rebecca Ramnauth** </br>
Computer Science PhD Student </br>
Yale University | AKW 507 | [rramnauth2220.github.io](rramnauth2220.github.io) </br>
