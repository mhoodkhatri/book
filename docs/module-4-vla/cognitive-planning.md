---
sidebar_position: 3
title: "Chapter 2: Cognitive Planning with LLMs"
description: "Implement language model-based planning for robot tasks. Translate high-level instructions into executable action sequences using LLMs and prompt engineering."
keywords: [cognitive planning, LLM, task planning, action sequencing, robotics AI, prompt engineering, LangChain]
---

# Chapter 2: Cognitive Planning with LLMs

**Teaching Your Robot to Think**

---

## Chapter Overview

In Chapter 1, your robot learned to hear. It can convert "Go to the kitchen and bring me a glass of water" into text. But text is not action. How does the robot know what "kitchen" means? How does it break down "bring me a glass of water" into individual steps?

This is where **cognitive planning** comes in. We will use Large Language Models (LLMs) to give your robot the ability to understand natural language and plan sequences of actions.

By the end of this chapter, your robot will be able to:
- Understand complex, multi-step instructions
- Break down high-level goals into executable action sequences
- Handle ambiguity and ask clarifying questions
- Recover from errors and adapt plans

### What You Will Learn

- How LLMs can be used for robotic task planning
- Prompt engineering techniques for reliable action generation
- Grounding language in robot capabilities (action spaces)
- Building a complete planning pipeline with ROS 2 integration
- Handling edge cases: ambiguity, failures, and safety

### Prerequisites

- [ ] Completed Chapter 1 (Speech to Action with Whisper)
- [ ] Basic understanding of transformer models (helpful but not required)
- [ ] Python programming experience
- [ ] Access to an LLM API (OpenAI, Anthropic, or local models)

---

## The Challenge: From Language to Action

Consider this command: "Clean up the living room."

A human understands this immediately. But what does it actually mean for a robot?

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     HUMAN vs ROBOT UNDERSTANDING                         │
│                                                                          │
│   Human hears: "Clean up the living room"                               │
│                                                                          │
│   Human thinks:                           Robot needs:                   │
│   - Look for things out of place          - Navigate to living_room     │
│   - Pick them up                          - Execute scan_for_objects    │
│   - Put them where they belong            - For each object:            │
│   - Maybe vacuum?                           - pick_object(obj_id)       │
│                                             - navigate_to(storage_loc)  │
│                                             - place_object()            │
│                                           - Return to start             │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

The gap between natural language and robot commands is enormous. Traditional approaches required programmers to anticipate every possible command and write explicit code for each. LLMs offer a different approach: they can understand intent and generate plans on the fly.

---

## LLMs for Robotics: The Key Insight

Large Language Models like GPT-4, Claude, and Llama have been trained on massive amounts of text—including robotics papers, programming tutorials, and task descriptions. They have developed an implicit understanding of:

- **Task decomposition**: Breaking complex tasks into steps
- **Spatial reasoning**: Understanding concepts like "next to," "on top of," "behind"
- **Object affordances**: Knowing that cups hold liquid, chairs are for sitting
- **Common sense**: Understanding that you need to open a door before walking through it

The key insight is: **we do not need to teach the LLM about tasks—we need to teach it about our robot**.

### The Action Space

Every robot has a limited set of things it can do. This is called the **action space**. For a mobile manipulator, it might include:

| Action | Parameters | Description |
|--------|------------|-------------|
| `navigate(location)` | location: string | Move to a named location |
| `pick(object)` | object: string | Pick up an object |
| `place(location)` | location: string | Place held object at location |
| `open(object)` | object: string | Open a door, drawer, or container |
| `close(object)` | object: string | Close a door, drawer, or container |
| `scan()` | none | Look around and identify objects |
| `say(message)` | message: string | Speak to the user |

The LLM needs to know:
1. What actions are available
2. What each action does
3. What the robot currently perceives

With this information, the LLM can translate any natural language command into a sequence of available actions.

---

## Building the Planning System

Let us build a complete cognitive planning system step by step. This is a complex system, so we will break it into manageable pieces.

### System Architecture Overview

Before diving into code, understand how the pieces fit together:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       COGNITIVE PLANNING SYSTEM                          │
│                                                                          │
│   User Command: "Get me a drink from the fridge"                        │
│                                │                                         │
│                                ▼                                         │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │  STEP 1: CONTEXT BUILDER                                         │   │
│   │  Gathers: Robot location, what it's holding, what it can see     │   │
│   └──────────────────────────────┬──────────────────────────────────┘   │
│                                  │                                       │
│                                  ▼                                       │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │  STEP 2: LLM PLANNER                                             │   │
│   │  Input: Context + Command → Output: List of actions              │   │
│   └──────────────────────────────┬──────────────────────────────────┘   │
│                                  │                                       │
│                                  ▼                                       │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │  STEP 3: PLAN VALIDATOR                                          │   │
│   │  Checks: Are all actions valid? Are parameters correct?          │   │
│   └──────────────────────────────┬──────────────────────────────────┘   │
│                                  │                                       │
│                                  ▼                                       │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │  STEP 4: PLAN EXECUTOR                                           │   │
│   │  Runs each action via ROS 2, monitors success/failure            │   │
│   └─────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

---

### Part 1: Define What Your Robot Can Do (Action Space)

The **action space** is the list of everything your robot can physically do. The LLM can only generate plans using these actions.

#### Why Define an Action Space?

Think of it like a menu at a restaurant. The LLM is the customer, and it can only order items on the menu. If "fly" is not on the menu, the robot will not try to fly.

```python
# Example action space (what the robot CAN do)
- navigate(location)     # Move to a place
- pick(object)          # Pick something up
- place(location)       # Put something down
- open(object)          # Open a door/drawer
- say(message)          # Speak to user

# NOT in action space (robot CANNOT do these)
- fly()
- teleport()
- read_minds()
```

#### Building the Action Space - Step by Step

**First, define the types of parameters actions can accept:**

```python
#!/usr/bin/env python3
"""action_space.py - Define what your robot can do."""

from dataclasses import dataclass
from typing import List, Dict, Any
from enum import Enum


class ParameterType(Enum):
    """What kinds of values can actions accept?"""
    STRING = "string"      # Any text: "hello", "kitchen"
    NUMBER = "number"      # Numbers: 5, 3.14
    BOOLEAN = "boolean"    # True or False
    LOCATION = "location"  # A named place: "kitchen", "bedroom"
    OBJECT = "object"      # A thing: "cup", "door"
```

**Next, define what a parameter looks like:**

```python
@dataclass
class ActionParameter:
    """
    One parameter that an action accepts.

    Example: navigate(location) has one parameter:
    - name: "location"
    - param_type: LOCATION
    - description: "Where to go"
    - required: True (must be provided)
    """
    name: str                    # Parameter name
    param_type: ParameterType    # What type of value
    description: str             # Explanation for the LLM
    required: bool = True        # Is it mandatory?
    default: Any = None          # Default value if not required
```

**Now define what an action looks like:**

```python
@dataclass
class Action:
    """
    One thing the robot can do.

    Example: The "pick" action
    - name: "pick"
    - description: "Pick up an object"
    - parameters: [object to pick]
    - preconditions: [must be near object, gripper must be empty]
    - effects: [robot is now holding the object]
    """
    name: str                      # Action name
    description: str               # What it does (for LLM)
    parameters: List[ActionParameter]  # Required inputs
    preconditions: List[str]       # What must be true before
    effects: List[str]             # What changes after

    def to_prompt_string(self) -> str:
        """Format this action for the LLM prompt."""
        params = ", ".join(
            f"{p.name}: {p.param_type.value}"
            for p in self.parameters
        )
        return f"- {self.name}({params}): {self.description}"
```

**Finally, create the complete action space:**

```python
class RobotActionSpace:
    """
    The complete list of what your robot can do.
    This is the "menu" the LLM orders from.
    """

    def __init__(self):
        self.actions: Dict[str, Action] = {}
        self._define_actions()

    def _define_actions(self):
        """Define each action the robot can perform."""

        # === NAVIGATION ACTIONS ===
        self.actions["navigate"] = Action(
            name="navigate",
            description="Move the robot to a named location",
            parameters=[
                ActionParameter(
                    name="location",
                    param_type=ParameterType.LOCATION,
                    description="Where to go (e.g., 'kitchen', 'bedroom')"
                )
            ],
            preconditions=["path is clear"],
            effects=["robot is at the specified location"]
        )

        self.actions["approach"] = Action(
            name="approach",
            description="Move close to a specific object",
            parameters=[
                ActionParameter(
                    name="object",
                    param_type=ParameterType.OBJECT,
                    description="The object to approach"
                )
            ],
            preconditions=["object is visible"],
            effects=["robot is within reach of object"]
        )

        # === MANIPULATION ACTIONS ===
        self.actions["pick"] = Action(
            name="pick",
            description="Pick up an object with the gripper",
            parameters=[
                ActionParameter(
                    name="object",
                    param_type=ParameterType.OBJECT,
                    description="The object to pick up"
                )
            ],
            preconditions=["robot is near object", "gripper is empty"],
            effects=["robot is holding the object"]
        )

        self.actions["place"] = Action(
            name="place",
            description="Put down the held object",
            parameters=[
                ActionParameter(
                    name="location",
                    param_type=ParameterType.STRING,
                    description="Where to place it (e.g., 'table')"
                )
            ],
            preconditions=["robot is holding something"],
            effects=["object is at location", "gripper is empty"]
        )

        # === INTERACTION ACTIONS ===
        self.actions["open"] = Action(
            name="open",
            description="Open a door, drawer, or container",
            parameters=[
                ActionParameter(
                    name="object",
                    param_type=ParameterType.OBJECT,
                    description="What to open (e.g., 'fridge_door')"
                )
            ],
            preconditions=["robot is near object", "object is closed"],
            effects=["object is open"]
        )

        self.actions["close"] = Action(
            name="close",
            description="Close a door, drawer, or container",
            parameters=[
                ActionParameter(
                    name="object",
                    param_type=ParameterType.OBJECT,
                    description="What to close"
                )
            ],
            preconditions=["robot is near object", "object is open"],
            effects=["object is closed"]
        )

        # === PERCEPTION ACTIONS ===
        self.actions["scan"] = Action(
            name="scan",
            description="Look around and identify objects",
            parameters=[],  # No parameters needed
            preconditions=[],
            effects=["robot knows what objects are visible"]
        )

        self.actions["find"] = Action(
            name="find",
            description="Search for a specific object",
            parameters=[
                ActionParameter(
                    name="object",
                    param_type=ParameterType.STRING,
                    description="What to look for"
                )
            ],
            preconditions=[],
            effects=["robot knows where object is (if found)"]
        )

        # === COMMUNICATION ACTIONS ===
        self.actions["say"] = Action(
            name="say",
            description="Speak a message to the user",
            parameters=[
                ActionParameter(
                    name="message",
                    param_type=ParameterType.STRING,
                    description="What to say"
                )
            ],
            preconditions=[],
            effects=["user has heard the message"]
        )

        self.actions["ask"] = Action(
            name="ask",
            description="Ask the user a question",
            parameters=[
                ActionParameter(
                    name="question",
                    param_type=ParameterType.STRING,
                    description="The question to ask"
                )
            ],
            preconditions=[],
            effects=["robot has user's answer"]
        )

        # === CONTROL ACTIONS ===
        self.actions["wait"] = Action(
            name="wait",
            description="Wait for a duration",
            parameters=[
                ActionParameter(
                    name="seconds",
                    param_type=ParameterType.NUMBER,
                    description="How long to wait"
                )
            ],
            preconditions=[],
            effects=["time has passed"]
        )
```

**Add helper methods to the class:**

```python
    def get_prompt_description(self) -> str:
        """
        Generate text describing all actions for the LLM.
        The LLM reads this to know what it can use.
        """
        lines = ["Available actions:"]
        for action in self.actions.values():
            lines.append(action.to_prompt_string())
        return "\n".join(lines)

    def validate_action(self, action_name: str, parameters: Dict) -> tuple:
        """
        Check if an action call is valid.
        Returns: (is_valid, error_message)
        """
        # Check action exists
        if action_name not in self.actions:
            return False, f"Unknown action: {action_name}"

        action = self.actions[action_name]

        # Check all required parameters are provided
        for param in action.parameters:
            if param.required and param.name not in parameters:
                return False, f"Missing required parameter: {param.name}"

        return True, None


# Create the action space (used throughout the system)
ROBOT_ACTIONS = RobotActionSpace()
```

**Testing the action space:**

```python
# What does the action space look like to the LLM?
print(ROBOT_ACTIONS.get_prompt_description())

# Output:
# Available actions:
# - navigate(location: location): Move the robot to a named location
# - approach(object: object): Move close to a specific object
# - pick(object: object): Pick up an object with the gripper
# - place(location: string): Put down the held object
# - open(object: object): Open a door, drawer, or container
# - close(object: object): Close a door, drawer, or container
# - scan(): Look around and identify objects
# - find(object: string): Search for a specific object
# - say(message: string): Speak a message to the user
# - ask(question: string): Ask the user a question
# - wait(seconds: number): Wait for a duration
```

---

### Part 2: Build the Prompt System

The **prompt** is how we tell the LLM what to do. A well-designed prompt is crucial for getting good plans.

#### What Goes in a Prompt?

The LLM needs to know:
1. **Instructions**: How to format the output, what rules to follow
2. **Available Actions**: What the robot can do (from our action space)
3. **Current State**: Where is the robot? What is it holding? What can it see?
4. **The Command**: What does the user want?

```
┌─────────────────────────────────────────────────────────────────────┐
│                      PROMPT STRUCTURE                                │
│                                                                      │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ 1. SYSTEM INSTRUCTIONS                                       │    │
│  │    "You are a robot planner. Follow these rules..."         │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                           │                                          │
│                           ▼                                          │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ 2. AVAILABLE ACTIONS                                         │    │
│  │    "You can use: navigate(), pick(), place()..."            │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                           │                                          │
│                           ▼                                          │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ 3. CURRENT STATE                                             │    │
│  │    "Robot is in living room, holding nothing..."            │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                           │                                          │
│                           ▼                                          │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ 4. USER COMMAND                                              │    │
│  │    "Get me a drink from the fridge"                         │    │
│  └─────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────┘
```

#### Step 1: Define State Data Structures

First, we need ways to represent the robot's current situation:

```python
#!/usr/bin/env python3
"""prompt_builder.py - Build prompts for the LLM planner."""

from typing import Dict, List, Optional
from dataclasses import dataclass
from action_space import ROBOT_ACTIONS


@dataclass
class RobotState:
    """
    What we know about the robot right now.

    Example:
    - Robot is in the kitchen
    - It's holding a cup
    - It can see the fridge, table, and chair
    """
    location: str                    # Where is the robot?
    holding: Optional[str]           # What is it holding? (None = empty)
    visible_objects: List[str]       # What can it see?
    known_locations: List[str]       # What places does it know?
    battery_level: float             # 0.0 to 1.0


@dataclass
class EnvironmentState:
    """
    What we know about the environment.

    Example:
    - The cup is on the table
    - The fridge door is open
    """
    objects: Dict[str, str]      # object -> location (e.g., "cup": "table")
    open_containers: List[str]   # What's currently open?
    occupied_locations: List[str] # Where are obstacles?
```

#### Step 2: Create the System Instructions

This tells the LLM how to behave:

```python
SYSTEM_PROMPT = """You are a robot task planner. Your job is to convert natural language commands into sequences of robot actions.

IMPORTANT RULES:
1. Only use actions from the provided action list
2. Each action must be on its own line: action_name(param="value")
3. Think step by step about what needs to happen
4. Consider preconditions - you cannot pick something up if not near it
5. If a command is ambiguous, use ask() to clarify
6. Always be safe - never harm humans or damage property
7. If you need to find something, use find() or scan() first

OUTPUT FORMAT:
First, briefly explain your reasoning (1-2 sentences).
Then output the plan as a numbered list of actions.

Example:
Reasoning: To get water, I need to go to kitchen, find a cup, and bring it back.
Plan:
1. navigate(location="kitchen")
2. scan()
3. find(object="cup")
4. approach(object="cup")
5. pick(object="cup")
6. say(message="I have the cup.")
"""
```

**Key rules explained:**
- **Rule 1**: Prevents the LLM from inventing impossible actions
- **Rule 4**: Ensures logical ordering (can't pick up what you're not near)
- **Rule 5**: Makes the robot ask when confused instead of guessing
- **Rule 7**: Ensures perception before manipulation

#### Step 3: Build the Prompt

Now we combine everything into one prompt:

```python
class PromptBuilder:
    """Assembles complete prompts for the LLM."""

    def __init__(self):
        self.action_space = ROBOT_ACTIONS

    def build_prompt(
        self,
        user_command: str,
        robot_state: RobotState,
        environment_state: Optional[EnvironmentState] = None
    ) -> str:
        """
        Build a complete prompt for the LLM.

        Args:
            user_command: What the user said ("get me a drink")
            robot_state: Current robot situation
            environment_state: Optional environment info

        Returns:
            Complete prompt string ready for the LLM
        """
        sections = []

        # Part 1: Instructions
        sections.append(SYSTEM_PROMPT)

        # Part 2: What actions are available
        sections.append("\n" + self.action_space.get_prompt_description())

        # Part 3: Current state
        state_text = self._build_state_section(robot_state, environment_state)
        sections.append("\n" + state_text)

        # Part 4: The command
        sections.append(f"\nUser command: {user_command}")
        sections.append("\nGenerate a plan to accomplish this command.")

        return "\n".join(sections)
```

#### Step 4: Format the State Information

```python
    def _build_state_section(
        self,
        robot_state: RobotState,
        environment_state: Optional[EnvironmentState]
    ) -> str:
        """Create a text description of the current state."""
        lines = ["Current state:"]

        # Robot information
        lines.append(f"- Robot location: {robot_state.location}")

        if robot_state.holding:
            lines.append(f"- Robot is holding: {robot_state.holding}")
        else:
            lines.append("- Robot gripper is empty")

        visible = ', '.join(robot_state.visible_objects) or 'nothing'
        lines.append(f"- Visible objects: {visible}")

        locations = ', '.join(robot_state.known_locations)
        lines.append(f"- Known locations: {locations}")

        lines.append(f"- Battery level: {robot_state.battery_level:.0%}")

        # Environment information (if provided)
        if environment_state:
            if environment_state.open_containers:
                open_stuff = ', '.join(environment_state.open_containers)
                lines.append(f"- Open containers: {open_stuff}")

            if environment_state.objects:
                lines.append("- Known object locations:")
                for obj, loc in environment_state.objects.items():
                    lines.append(f"    {obj} is at {loc}")

        return "\n".join(lines)
```

#### Testing the Prompt Builder

```python
# Create the builder
builder = PromptBuilder()

# Define current robot state
robot_state = RobotState(
    location="living_room",
    holding=None,
    visible_objects=["couch", "coffee_table", "tv_remote"],
    known_locations=["living_room", "kitchen", "bedroom"],
    battery_level=0.85
)

# Build the prompt
prompt = builder.build_prompt(
    user_command="Get me a drink from the fridge",
    robot_state=robot_state
)

print(prompt)
```

**Output (what the LLM sees):**
```
You are a robot task planner. Your job is to convert natural language...

Available actions:
- navigate(location: location): Move the robot to a named location
- pick(object: object): Pick up an object with the gripper
...

Current state:
- Robot location: living_room
- Robot gripper is empty
- Visible objects: couch, coffee_table, tv_remote
- Known locations: living_room, kitchen, bedroom
- Battery level: 85%

User command: Get me a drink from the fridge

Generate a plan to accomplish this command.
```

---

### Part 3: Connect to an LLM

Now we connect to a Large Language Model to generate plans. We will support multiple LLM providers.

#### Understanding LLM Providers

Different LLM providers have different APIs:

| Provider | Models | Cost | Pros | Cons |
|----------|--------|------|------|------|
| **OpenAI** | GPT-4, GPT-3.5 | Paid API | Best quality | Requires internet, costs money |
| **Anthropic** | Claude 3 | Paid API | Good reasoning | Requires internet, costs money |
| **Local (Ollama)** | Llama, Mistral | Free | Private, free | Needs good hardware |

#### Step 1: Define Data Structures for Plans

```python
#!/usr/bin/env python3
"""llm_planner.py - Generate robot plans using LLMs."""

import re
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from abc import ABC, abstractmethod

from prompt_builder import PromptBuilder, RobotState, EnvironmentState
from action_space import ROBOT_ACTIONS


@dataclass
class PlannedAction:
    """
    One action in the plan.

    Example:
    - action_name: "navigate"
    - parameters: {"location": "kitchen"}
    - step_number: 1
    """
    action_name: str
    parameters: Dict[str, str]
    step_number: int


@dataclass
class Plan:
    """
    A complete plan from the LLM.

    Contains:
    - reasoning: Why the LLM chose this plan
    - actions: List of actions to execute
    - raw_response: Original LLM output (for debugging)
    """
    reasoning: str
    actions: List[PlannedAction]
    raw_response: str
```

#### Step 2: Create LLM Provider Interface

We use an abstract class so we can swap providers easily:

```python
class LLMProvider(ABC):
    """
    Base class for LLM providers.
    All providers must implement the generate() method.
    """

    @abstractmethod
    def generate(self, prompt: str) -> str:
        """Send prompt to LLM, return the response text."""
        pass
```

#### Step 3: Implement OpenAI Provider

```python
class OpenAIProvider(LLMProvider):
    """Use OpenAI's GPT models."""

    def __init__(self, model: str = "gpt-4", api_key: Optional[str] = None):
        import openai
        self.client = openai.OpenAI(api_key=api_key)
        self.model = model

    def generate(self, prompt: str) -> str:
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1,  # Low = more consistent/predictable
            max_tokens=1000
        )
        return response.choices[0].message.content
```

**Why low temperature?** Higher temperature = more creative/random. For robot planning, we want consistent, reliable plans.

#### Step 4: Implement Anthropic Provider

```python
class AnthropicProvider(LLMProvider):
    """Use Anthropic's Claude models."""

    def __init__(self, model: str = "claude-3-sonnet-20240229", api_key: Optional[str] = None):
        import anthropic
        self.client = anthropic.Anthropic(api_key=api_key)
        self.model = model

    def generate(self, prompt: str) -> str:
        response = self.client.messages.create(
            model=self.model,
            max_tokens=1000,
            messages=[{"role": "user", "content": prompt}]
        )
        return response.content[0].text
```

#### Step 5: Implement Local Provider (Ollama)

For running LLMs on your own machine (free, private):

```python
class LocalLLMProvider(LLMProvider):
    """Use local LLMs via Ollama."""

    def __init__(self, base_url: str = "http://localhost:11434", model: str = "llama2"):
        import requests
        self.base_url = base_url
        self.model = model
        self.session = requests.Session()

    def generate(self, prompt: str) -> str:
        response = self.session.post(
            f"{self.base_url}/api/generate",
            json={
                "model": self.model,
                "prompt": prompt,
                "stream": False
            }
        )
        return response.json()["response"]
```

**Setting up Ollama:**
```bash
# Install Ollama (on Ubuntu)
curl -fsSL https://ollama.com/install.sh | sh

# Download a model
ollama pull llama2

# It's now ready to use!
```

#### Step 6: Build the Task Planner

Now we combine everything:

```python
class TaskPlanner:
    """
    Generates robot action plans using LLMs.

    Flow:
    1. Build prompt with current state
    2. Send to LLM
    3. Parse the response
    4. Validate the plan
    5. Retry if needed
    """

    def __init__(self, llm_provider: LLMProvider):
        self.llm = llm_provider
        self.prompt_builder = PromptBuilder()
        self.action_space = ROBOT_ACTIONS
        self.max_retries = 3  # Try up to 3 times

    def plan(
        self,
        command: str,
        robot_state: RobotState,
        environment_state: Optional[EnvironmentState] = None
    ) -> Plan:
        """
        Generate a plan for a user command.

        Args:
            command: What the user said ("get me a drink")
            robot_state: Current robot situation
            environment_state: Optional environment info

        Returns:
            Plan object with actions to execute
        """
        # Build the prompt
        prompt = self.prompt_builder.build_prompt(
            user_command=command,
            robot_state=robot_state,
            environment_state=environment_state
        )

        # Try to get a valid plan (with retries)
        for attempt in range(self.max_retries):
            try:
                # Send to LLM
                response = self.llm.generate(prompt)

                # Parse the response into a Plan object
                plan = self._parse_response(response)

                # Check if the plan is valid
                is_valid, errors = self._validate_plan(plan)

                if is_valid:
                    return plan  # Success!
                else:
                    # Tell LLM about errors and retry
                    prompt += f"\n\nErrors found: {errors}. Please fix."

            except Exception as e:
                if attempt == self.max_retries - 1:
                    raise RuntimeError(f"Failed after {self.max_retries} attempts: {e}")

        raise RuntimeError("Could not generate valid plan")
```

#### Step 7: Parse LLM Output

The LLM returns text. We need to extract the actions:

```python
    def _parse_response(self, response: str) -> Plan:
        """
        Convert LLM text output into a Plan object.

        Expected format from LLM:
        Reasoning: I need to go to the kitchen first...
        Plan:
        1. navigate(location="kitchen")
        2. open(object="fridge")
        3. ...
        """
        # Extract the reasoning section
        reasoning_match = re.search(
            r"Reasoning:\s*(.+?)(?=Plan:|$)",
            response,
            re.DOTALL  # Allow . to match newlines
        )
        reasoning = reasoning_match.group(1).strip() if reasoning_match else ""

        # Extract each action using regex
        # Pattern matches: "1. navigate(location="kitchen")"
        actions = []
        action_pattern = r"(\d+)\.\s*(\w+)\(([^)]*)\)"

        for match in re.finditer(action_pattern, response):
            step_num = int(match.group(1))    # "1"
            action_name = match.group(2)       # "navigate"
            params_str = match.group(3)        # 'location="kitchen"'

            # Parse the parameters
            parameters = {}
            if params_str:
                # Find all param="value" or param='value' pairs
                param_pattern = r'(\w+)\s*=\s*["\']([^"\']*)["\']'
                for param_match in re.finditer(param_pattern, params_str):
                    param_name = param_match.group(1)   # "location"
                    param_value = param_match.group(2)  # "kitchen"
                    parameters[param_name] = param_value

            actions.append(PlannedAction(
                action_name=action_name,
                parameters=parameters,
                step_number=step_num
            ))

        return Plan(
            reasoning=reasoning,
            actions=actions,
            raw_response=response
        )
```

#### Step 8: Validate the Plan

Make sure the LLM didn't generate invalid actions:

```python
    def _validate_plan(self, plan: Plan) -> Tuple[bool, List[str]]:
        """
        Check if a plan can actually be executed.

        Checks:
        1. Plan has at least one action
        2. All actions exist in action space
        3. All required parameters are provided
        """
        errors = []

        # Check for empty plan
        if not plan.actions:
            errors.append("Plan contains no actions")
            return False, errors

        # Validate each action
        for action in plan.actions:
            is_valid, error = self.action_space.validate_action(
                action.action_name,
                action.parameters
            )
            if not is_valid:
                errors.append(f"Step {action.step_number}: {error}")

        return len(errors) == 0, errors
```

#### Testing the Planner

```python
# Choose your provider
# provider = OpenAIProvider(model="gpt-4")         # Paid, best quality
# provider = AnthropicProvider()                    # Paid, good quality
provider = LocalLLMProvider(model="llama2")        # Free, runs locally

# Create the planner
planner = TaskPlanner(provider)

# Define current state
robot_state = RobotState(
    location="living_room",
    holding=None,
    visible_objects=["couch", "coffee_table"],
    known_locations=["living_room", "kitchen", "bedroom"],
    battery_level=0.9
)

# Generate a plan!
plan = planner.plan(
    command="Get me a drink from the fridge",
    robot_state=robot_state
)

# Print the result
print("Reasoning:", plan.reasoning)
print("\nPlan:")
for action in plan.actions:
    params = ", ".join(f'{k}="{v}"' for k, v in action.parameters.items())
    print(f"  {action.step_number}. {action.action_name}({params})")
```

**Example output:**
```
Reasoning: To get a drink from the fridge, I need to navigate to the kitchen,
open the fridge, find a drink, pick it up, close the fridge, and return.

Plan:
  1. navigate(location="kitchen")
  2. approach(object="fridge")
  3. open(object="fridge_door")
  4. find(object="drink")
  5. pick(object="drink")
  6. close(object="fridge_door")
  7. navigate(location="living_room")
  8. say(message="Here is your drink!")
```

---

### Part 4: Execute Plans with ROS 2

Now we need to actually run the plans on the robot. This is where planning meets action.

#### The Plan Executor Concept

The executor takes each action in the plan and runs it via ROS 2:

```
Plan:                           Execution:
1. navigate("kitchen")    →    Send goal to Nav2 → Wait → Success
2. open("fridge")         →    Send to manipulation → Wait → Success
3. pick("drink")          →    Send to gripper → Wait → Success
4. ...
```

#### Step 1: Define Result Types

```python
#!/usr/bin/env python3
"""plan_executor.py - Execute plans via ROS 2."""

from enum import Enum
from dataclasses import dataclass
from typing import Dict, List, Optional
from llm_planner import Plan, PlannedAction


class ActionStatus(Enum):
    """Possible outcomes of an action."""
    PENDING = "pending"       # Not started yet
    EXECUTING = "executing"   # Currently running
    SUCCEEDED = "succeeded"   # Completed successfully
    FAILED = "failed"         # Something went wrong
    CANCELLED = "cancelled"   # User stopped it


@dataclass
class ActionResult:
    """What happened when we tried to execute an action."""
    action: PlannedAction    # Which action was attempted
    status: ActionStatus     # Did it work?
    message: str             # Details (e.g., "Arrived at kitchen")
    duration_seconds: float  # How long it took
```

#### Step 2: Create the Executor Node

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class PlanExecutor(Node):
    """
    Executes LLM-generated plans using ROS 2.

    For each action in the plan, calls the appropriate
    ROS 2 service or action server.
    """

    def __init__(self):
        super().__init__('plan_executor')

        # Set up Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Publisher for speech output
        self.speech_pub = self.create_publisher(String, 'speech/output', 10)

        # Store known locations (would normally load from config)
        self.locations = {
            "kitchen": (5.0, 2.0),
            "living_room": (0.0, 0.0),
            "bedroom": (-3.0, 4.0),
        }

        # Execution state
        self.is_executing = False
        self.results = []

        self.get_logger().info('Plan executor ready')
```

#### Step 3: The Main Execution Loop

```python
    async def execute_plan(self, plan: Plan) -> List[ActionResult]:
        """
        Execute all actions in a plan, one by one.

        Stops early if any action fails.
        """
        self.is_executing = True
        self.results = []

        self.get_logger().info(f'Starting plan with {len(plan.actions)} actions')

        for action in plan.actions:
            # Check if cancelled
            if not self.is_executing:
                self.get_logger().info('Execution cancelled')
                break

            # Log what we're doing
            self.get_logger().info(
                f'Step {action.step_number}: {action.action_name}'
            )

            # Execute the action
            result = await self._execute_action(action)
            self.results.append(result)

            # Stop if it failed
            if result.status == ActionStatus.FAILED:
                self.get_logger().error(f'Failed: {result.message}')
                break

        self.is_executing = False
        return self.results
```

#### Step 4: Dispatch Actions to Handlers

Each action type has its own handler:

```python
    async def _execute_action(self, action: PlannedAction) -> ActionResult:
        """Route an action to the appropriate handler."""
        import time
        start = time.time()

        # Map action names to handler functions
        handlers = {
            "navigate": self._do_navigate,
            "approach": self._do_approach,
            "pick": self._do_pick,
            "place": self._do_place,
            "open": self._do_open,
            "close": self._do_close,
            "scan": self._do_scan,
            "find": self._do_find,
            "say": self._do_say,
            "ask": self._do_ask,
            "wait": self._do_wait,
        }

        handler = handlers.get(action.action_name)
        if not handler:
            return ActionResult(
                action=action,
                status=ActionStatus.FAILED,
                message=f"Unknown action: {action.action_name}",
                duration_seconds=time.time() - start
            )

        # Call the handler
        status, message = await handler(action.parameters)

        return ActionResult(
            action=action,
            status=status,
            message=message,
            duration_seconds=time.time() - start
        )
```

#### Step 5: Implement Key Action Handlers

**Navigation (using Nav2):**

```python
    async def _do_navigate(self, params: Dict) -> tuple:
        """Navigate to a named location using Nav2."""
        location = params.get("location")

        # Check if we know this location
        if location not in self.locations:
            return ActionStatus.FAILED, f"Unknown location: {location}"

        x, y = self.locations[location]

        # Create the goal pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # Wait for Nav2 to be ready
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return ActionStatus.FAILED, "Nav2 not available"

        # Send the goal and wait for result
        result = await self.nav_client.send_goal_async(goal)
        if not result.accepted:
            return ActionStatus.FAILED, "Goal rejected by Nav2"

        # Wait for navigation to complete
        final = await result.get_result_async()
        if final.status == 4:  # SUCCEEDED
            return ActionStatus.SUCCEEDED, f"Arrived at {location}"
        else:
            return ActionStatus.FAILED, "Navigation failed"
```

**Speech output:**

```python
    async def _do_say(self, params: Dict) -> tuple:
        """Speak a message using text-to-speech."""
        message = params.get("message", "")

        # Publish to speech topic
        msg = String()
        msg.data = message
        self.speech_pub.publish(msg)

        self.get_logger().info(f'Said: "{message}"')
        return ActionStatus.SUCCEEDED, f"Said: {message}"
```

**Placeholder handlers (for simulation):**

```python
    async def _do_approach(self, params: Dict) -> tuple:
        """Move close to an object (placeholder)."""
        obj = params.get("object")
        # In real system: use perception + motion planning
        self.get_logger().info(f"Approaching {obj}")
        return ActionStatus.SUCCEEDED, f"Approached {obj}"

    async def _do_pick(self, params: Dict) -> tuple:
        """Pick up an object (placeholder)."""
        obj = params.get("object")
        self.get_logger().info(f"Picking up {obj}")
        return ActionStatus.SUCCEEDED, f"Picked up {obj}"

    async def _do_place(self, params: Dict) -> tuple:
        """Place an object (placeholder)."""
        loc = params.get("location")
        return ActionStatus.SUCCEEDED, f"Placed at {loc}"

    async def _do_open(self, params: Dict) -> tuple:
        """Open something (placeholder)."""
        obj = params.get("object")
        return ActionStatus.SUCCEEDED, f"Opened {obj}"

    async def _do_close(self, params: Dict) -> tuple:
        """Close something (placeholder)."""
        obj = params.get("object")
        return ActionStatus.SUCCEEDED, f"Closed {obj}"

    async def _do_scan(self, params: Dict) -> tuple:
        """Look around (placeholder)."""
        return ActionStatus.SUCCEEDED, "Scan complete"

    async def _do_find(self, params: Dict) -> tuple:
        """Search for object (placeholder)."""
        obj = params.get("object")
        return ActionStatus.SUCCEEDED, f"Found {obj}"

    async def _do_ask(self, params: Dict) -> tuple:
        """Ask user a question (placeholder)."""
        q = params.get("question")
        return ActionStatus.SUCCEEDED, f"Asked: {q}"

    async def _do_wait(self, params: Dict) -> tuple:
        """Wait for a duration."""
        import asyncio
        seconds = float(params.get("seconds", 1.0))
        await asyncio.sleep(seconds)
        return ActionStatus.SUCCEEDED, f"Waited {seconds}s"
```

---

## Advanced Techniques

### Chain-of-Thought Reasoning

For complex tasks, we can ask the LLM to think step by step:

```python
COT_SYSTEM_PROMPT = """You are a robot task planner that thinks carefully before acting.

When given a command, follow this process:
1. UNDERSTAND: What is the user really asking for?
2. CONSTRAINTS: What limitations do I have?
3. DECOMPOSE: Break the task into sub-tasks
4. SEQUENCE: Order the sub-tasks logically
5. PLAN: Convert sub-tasks to robot actions

Example:
User: "Make me a sandwich"

UNDERSTAND: User wants a sandwich prepared and delivered.
CONSTRAINTS: Robot can navigate and manipulate but cannot cook.
DECOMPOSE:
  - Get bread
  - Get ingredients
  - Assemble sandwich
  - Deliver to user
SEQUENCE: Must get items before assembling, assembly before delivery.
PLAN:
1. navigate(location="kitchen")
2. open(object="bread_box")
3. pick(object="bread")
...

Now respond to the user command with this thought process.
"""
```

### Handling Failures with Replanning

When actions fail, we can ask the LLM to replan:

```python
class ReplanningExecutor:
    """
    Executor that can replan when actions fail.
    """

    def __init__(self, planner: TaskPlanner, executor: PlanExecutor):
        self.planner = planner
        self.executor = executor
        self.max_replans = 3

    async def execute_with_replanning(
        self,
        command: str,
        robot_state: RobotState
    ) -> bool:
        """
        Execute command with automatic replanning on failure.
        """
        replan_count = 0

        while replan_count < self.max_replans:
            # Generate plan
            plan = self.planner.plan(command, robot_state)

            # Execute plan
            results = await self.executor.execute_plan(plan)

            # Check for failures
            failed = [r for r in results if r.status == ActionStatus.FAILED]

            if not failed:
                return True  # Success!

            # Prepare for replanning
            replan_count += 1
            failed_action = failed[0]

            # Update command with failure context
            command = f"""
            Original command: {command}

            The plan failed at step {failed_action.action.step_number}:
            Action: {failed_action.action.action_name}({failed_action.action.parameters})
            Error: {failed_action.message}

            Please create a new plan that avoids this failure.
            """

            # Update robot state based on successful actions
            # (implementation depends on your state tracking)

        return False  # Failed after max replans
```

### Grounding in Perception

The LLM needs to know what the robot actually sees:

```python
class PerceptionGrounding:
    """
    Ground LLM planning in actual perception data.
    """

    def __init__(self, node: Node):
        self.node = node
        self.detected_objects = []

        # Subscribe to object detection
        self.detection_sub = node.create_subscription(
            DetectedObjects,  # Custom message type
            'perception/objects',
            self._detection_callback,
            10
        )

    def _detection_callback(self, msg):
        """Update detected objects from perception."""
        self.detected_objects = [
            {
                "name": obj.label,
                "confidence": obj.confidence,
                "position": (obj.pose.position.x, obj.pose.position.y),
                "graspable": obj.confidence > 0.8
            }
            for obj in msg.objects
        ]

    def get_visible_objects_description(self) -> str:
        """
        Generate description for LLM prompt.
        """
        if not self.detected_objects:
            return "No objects currently visible."

        lines = ["Currently visible objects:"]
        for obj in self.detected_objects:
            graspable = "graspable" if obj["graspable"] else "not graspable"
            lines.append(
                f"  - {obj['name']} at position "
                f"({obj['position'][0]:.1f}, {obj['position'][1]:.1f}), "
                f"{graspable}"
            )

        return "\n".join(lines)
```

---

## Safety Considerations

LLMs can generate unsafe actions. We must add safety checks:

```python
class SafetyChecker:
    """
    Check plans for safety before execution.
    """

    def __init__(self):
        # Actions that need extra validation
        self.high_risk_actions = ["pick", "place", "open", "close"]

        # Forbidden action sequences
        self.forbidden_sequences = [
            ["pick", "navigate"],  # Don't move while holding fragile items
        ]

        # Maximum actions in a plan
        self.max_plan_length = 20

    def check_plan(self, plan: Plan) -> tuple:
        """
        Check if a plan is safe to execute.

        Returns:
            (is_safe: bool, issues: List[str])
        """
        issues = []

        # Check plan length
        if len(plan.actions) > self.max_plan_length:
            issues.append(f"Plan too long ({len(plan.actions)} actions)")

        # Check for forbidden sequences
        for i in range(len(plan.actions) - 1):
            current = plan.actions[i].action_name
            next_action = plan.actions[i + 1].action_name

            if [current, next_action] in self.forbidden_sequences:
                issues.append(
                    f"Forbidden sequence: {current} followed by {next_action}"
                )

        # Check for dangerous parameters
        for action in plan.actions:
            if action.action_name == "say":
                message = action.parameters.get("message", "")
                # Check for inappropriate content
                if self._contains_inappropriate_content(message):
                    issues.append(f"Inappropriate message content")

        return len(issues) == 0, issues

    def _contains_inappropriate_content(self, text: str) -> bool:
        """Check for inappropriate content."""
        # Simple keyword check - would be more sophisticated in production
        forbidden_words = ["hate", "kill", "harm"]
        return any(word in text.lower() for word in forbidden_words)
```

---

## Complete ROS 2 Integration

Here is the complete planning node that ties everything together:

```python
#!/usr/bin/env python3
"""
cognitive_planner_node.py
Complete ROS 2 node for LLM-based task planning.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

import asyncio
from concurrent.futures import ThreadPoolExecutor

from llm_planner import TaskPlanner, OpenAIProvider, LocalLLMProvider
from plan_executor import PlanExecutor
from prompt_builder import RobotState, EnvironmentState


class CognitivePlannerNode(Node):
    """
    ROS 2 node that provides cognitive planning capabilities.

    Subscribes to:
        - /speech/command: Voice commands from Whisper
        - /perception/objects: Detected objects

    Publishes to:
        - /planner/status: Planning and execution status
        - /speech/output: Speech output for user feedback
    """

    def __init__(self):
        super().__init__('cognitive_planner')

        # Parameters
        self.declare_parameter('llm_provider', 'openai')
        self.declare_parameter('llm_model', 'gpt-4')
        self.declare_parameter('local_llm_url', 'http://localhost:11434')

        # Set up LLM provider
        provider_name = self.get_parameter('llm_provider').value
        if provider_name == 'openai':
            model = self.get_parameter('llm_model').value
            self.llm_provider = OpenAIProvider(model=model)
        else:
            url = self.get_parameter('local_llm_url').value
            model = self.get_parameter('llm_model').value
            self.llm_provider = LocalLLMProvider(base_url=url, model=model)

        # Create planner
        self.planner = TaskPlanner(self.llm_provider)

        # Publishers
        self.status_pub = self.create_publisher(String, 'planner/status', 10)
        self.speech_pub = self.create_publisher(String, 'speech/output', 10)

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            'speech/command',
            self._command_callback,
            10
        )

        # Robot state (would be updated from actual robot)
        self.robot_state = RobotState(
            location="home",
            holding=None,
            visible_objects=[],
            known_locations=["home", "kitchen", "living_room", "bedroom"],
            battery_level=1.0
        )

        # Thread pool for async operations
        self.executor_pool = ThreadPoolExecutor(max_workers=2)

        self.get_logger().info('Cognitive planner node initialized')

    def _command_callback(self, msg: String):
        """Handle incoming voice commands."""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Run planning in thread pool
        self.executor_pool.submit(self._process_command, command)

    def _process_command(self, command: str):
        """Process a command (runs in thread pool)."""
        try:
            # Publish status
            self._publish_status(f"Planning: {command}")

            # Generate plan
            plan = self.planner.plan(command, self.robot_state)

            # Log the plan
            self.get_logger().info(f'Generated plan:')
            self.get_logger().info(f'Reasoning: {plan.reasoning}')
            for action in plan.actions:
                params = ", ".join(f'{k}="{v}"' for k, v in action.parameters.items())
                self.get_logger().info(f'  {action.step_number}. {action.action_name}({params})')

            # Announce plan to user
            self._say(f"I understand. {plan.reasoning}")

            # Execute plan
            self._publish_status("Executing plan...")

            # Note: Actual execution would use the PlanExecutor
            # This is simplified for demonstration
            for action in plan.actions:
                self._publish_status(f"Executing: {action.action_name}")
                # await executor.execute_action(action)

            self._publish_status("Plan completed")
            self._say("I have completed the task.")

        except Exception as e:
            self.get_logger().error(f'Planning failed: {e}')
            self._publish_status(f"Error: {e}")
            self._say("I'm sorry, I couldn't complete that task.")

    def _publish_status(self, status: str):
        """Publish planner status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _say(self, message: str):
        """Publish speech output."""
        msg = String()
        msg.data = message
        self.speech_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = CognitivePlannerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Hands-on Exercises

### Exercise 1: Basic Planning

1. Set up an LLM provider (local or API)
2. Define your robot's action space
3. Generate a plan for "Get me a drink"

```python
# Test your planner
planner = TaskPlanner(your_provider)
robot_state = RobotState(...)
plan = planner.plan("Get me a drink", robot_state)
print(plan.reasoning)
for action in plan.actions:
    print(f"  {action.action_name}({action.parameters})")
```

### Exercise 2: Handle Ambiguity

Test commands that are ambiguous:
- "Clean up" (clean what?)
- "Get the thing" (what thing?)
- "Go there" (where?)

Verify your planner uses `ask()` to clarify.

### Exercise 3: Failure Recovery

1. Simulate a failure during execution
2. Feed the failure back to the planner
3. Verify it generates a new plan

### Exercise 4: Full Integration

1. Connect Whisper (Chapter 1) to the cognitive planner
2. Speak a command
3. Watch the plan generated
4. (Optional) Execute on a simulated robot

---

## Summary

In this chapter, you learned to give your robot the ability to understand and plan:

| Concept | What You Learned |
|---------|------------------|
| **Action Spaces** | Defining what your robot can do |
| **Prompt Engineering** | Structuring prompts for reliable plans |
| **LLM Integration** | Connecting to various LLM providers |
| **Plan Parsing** | Converting LLM output to executable actions |
| **Plan Validation** | Ensuring plans are safe and valid |
| **Execution** | Running plans via ROS 2 actions |
| **Replanning** | Recovering from failures |

Your robot can now hear a command like "Get me a drink from the fridge" and generate a complete plan:
1. Navigate to kitchen
2. Approach fridge
3. Open fridge door
4. Find drink
5. Pick up drink
6. Close fridge door
7. Navigate back to user
8. Hand over drink

---

## What's Next

In Chapter 3, you will complete a capstone project that integrates everything:
- Speech recognition (Chapter 1)
- Cognitive planning (Chapter 2)
- Simulation and navigation (Modules 2-3)

You will build a complete voice-controlled robot system from end to end.

[Continue to Chapter 3: Capstone Project →](/docs/module-4-vla/capstone)

---

## Additional Resources

- [LangChain Documentation](https://python.langchain.com/) - Framework for LLM applications
- [OpenAI API Reference](https://platform.openai.com/docs/) - GPT models
- [Anthropic API Reference](https://docs.anthropic.com/) - Claude models
- [Ollama](https://ollama.ai/) - Run LLMs locally
- [SayCan Paper](https://say-can.github.io/) - Google's grounded LLM planning
- [Code as Policies](https://code-as-policies.github.io/) - LLM-generated robot code
