import json
import os
from pathlib import Path
from typing import Any, List, Union
from prompt_toolkit import prompt
from prompt_toolkit.validation import Validator
from prompt_toolkit.completion import FuzzyWordCompleter
from prompt_toolkit.application import Application
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.layout import Layout
from prompt_toolkit.layout.containers import HSplit, VSplit, Window
from prompt_toolkit.layout.controls import FormattedTextControl
from prompt_toolkit.widgets import TextArea, Label, Button, Dialog, Box, Frame, Checkbox
from prompt_toolkit.shortcuts import message_dialog, confirm


class DictionaryEditor:
    def __init__(self, data: dict = None, basepath: str = 'config'):
        self.dictionary = data or {}
        self.current_path = []
        self.current_index = 0
        self.basepath = basepath
        self.filename = self.get_default_path() if data else self.select_path()
        self.dictionary = self.load_dict() if data is None else data
        self.key_bindings = self.setup_key_bindings()

        # Initialize the application
        self.app = None
        self.container = None
        self.update_container()

    def get_default_path(self):
        i = 0
        while True:
            filename = os.path.join(self.basepath, f'untitled_{i}.json')
            if not os.path.exists(filename):
                return filename
            i += 1

    def load_dict(self) -> dict:
        """Load dictionary from a JSON file."""
        if os.path.exists(self.filename):
            with open(self.filename, 'r') as file:
                return json.load(file)
        return {}

    def save_dict(self):
        """Save the current dictionary to a JSON file."""
        filename = prompt("Save with filename: ", default=self.filename)
        if not filename.endswith('.json'):
            filename += '.json'
        filename = os.path.join(self.basepath, filename)
        with open(filename, 'w') as file:
            json.dump(self.dictionary, file, indent=4)
        message_dialog(title="Success", text=f"Saved to {filename}").run()

    def select_path(self) -> str:
        """Prompt the user to select a file path."""
        paths = [str(p.name) for p in Path(self.basepath).iterdir()]
        completer = FuzzyWordCompleter(paths)
        selected = prompt("Select path: ", completer=completer)
        return os.path.join(self.basepath, selected)

    def get_current_item(self):
        """Get the current item based on the path."""
        item = self.dictionary
        for key in self.current_path:
            item = item[key]
        return item

    def get_keys(self):
        """Get the keys or indices at the current level."""
        item = self.get_current_item()
        if isinstance(item, dict):
            return list(item.keys())
        elif isinstance(item, list):
            return list(range(len(item)))
        return []

    def update_container(self):
        """Update the container to reflect the current view."""
        item = self.get_current_item()
        keys = self.get_keys()
        rows = []
        
        def intValidator(s:str) -> bool:
            try:
                int(s)
                return True
            except ValueError:
                return False
            
        def floatValidator(s:str) -> bool:
            try:
                float(s)
                return True
            except ValueError:
                return False

        for i, key in enumerate(keys):
            value = item[key]
            is_selected = i == self.current_index
            key_text = f"> {key}" if is_selected else f"  {key}"
            value_text = str(value)

            editor = None
            if isinstance(value, str):
                editor = TextArea(value_text, False, focus_on_click=True)
            if isinstance(value, int):
                editor = TextArea(value_text, False, validator=Validator.from_callable(intValidator, 'Must be an integer'), focus_on_click=True)
            
            row = VSplit([
                Window(FormattedTextControl(key_text), width=20),
                Window(FormattedTextControl(value_text))
            ])
            rows.append(row)

        # Create a Frame with key-value pairs
        self.container = Frame(
            Box(HSplit(rows), padding=1, style="class:table"),
            title="Dictionary Editor",
            style="class:frame"
        )

    def edit_item(self):
        """Edit the currently selected item."""
        item = self.get_current_item()
        key = self.get_keys()[self.current_index]
        current_value = item[key]

        def value_validator(value: str) -> bool:
            """Validate if the input is of the correct type."""
            try:
                type(current_value)(value)
                return True
            except ValueError:
                return False

        # Get new value from the user
        new_value = prompt(
            f"Edit value for '{key}': ",
            default=str(current_value),
            validator=Validator.from_callable(
                value_validator, error_message="Invalid value type", move_cursor_to_end=True
            )
        )
        item[key] = type(current_value)(new_value)
        self.update_container()

    def setup_key_bindings(self) -> KeyBindings:
        """Define key bindings for navigation and actions."""
        bindings = KeyBindings()

        @bindings.add('up')
        def move_up(event):
            self.current_index = (self.current_index - 1) % len(self.get_keys())
            self.update_container()

        @bindings.add('down')
        def move_down(event):
            self.current_index = (self.current_index + 1) % len(self.get_keys())
            self.update_container()

        @bindings.add('right')
        def enter_nested(event):
            """Enter nested dictionary or list."""
            item = self.get_current_item()
            key = self.get_keys()[self.current_index]
            if isinstance(item[key], (dict, list)):
                self.current_path.append(key)
                self.current_index = 0
                self.update_container()

        @bindings.add('left')
        def go_back(event):
            """Go back to the parent level."""
            if self.current_path:
                self.current_path.pop()
                self.current_index = 0
                self.update_container()

        @bindings.add('e')
        def edit_current(event):
            self.edit_item()

        @bindings.add('s')
        def save_current(event):
            self.save_dict()

        @bindings.add('q')
        def quit_app(event):
            event.app.exit()

        return bindings

    def create_application(self) -> Application:
        """Create and run the prompt_toolkit application."""
        layout = Layout(container=self.container)
        return Application(layout=layout, mouse_support=True, key_bindings=self.key_bindings, full_screen=True)

    def run(self):
        """Run the application."""
        self.app = self.create_application()
        self.app.run()