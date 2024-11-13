import tkinter as tk
from tkinter import ttk

from configuration.configuration import Configuration
from domain.enums.manual_command_enum import ManualCommandEnum
from utils.enum_utils import EnumUtils

class ControlCenter:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.configuracao = Configuration.get_object()

        self.robot_id_combobox_mapping: 'dict[str, ttk.Combobox]' = {}
        self.robot_mapping_id_entry_mapping: 'dict[str, ttk.Entry]' = {}

        self.desired_keeper_id_combobox: ttk.Combobox = None
        self.defensor_id_combobox: ttk.Combobox = None
        self.attacker_id_combobox: ttk.Combobox = None

        self.robot_ids = ["0", "1", "2"]

        root.title("Control Center")

        self.create_top_frame()

        self.create_mappings_frame()

        self.create_ids_frame()

        self.create_manual_frame()

        self.create_replacement_frame()

    def main(self):
        self.root.mainloop()

    def create_top_frame(self):
        team_color = "Yellow" if self.configuracao.team_is_yellow_team else "Blue"
        is_left_team = self.configuracao.get_is_left_team()
        playing_side = "Left" if is_left_team else "Right"

        top_frame = tk.Frame(self.root)
        top_frame.grid(row=0, column=0, columnspan=3, padx=10, pady=5, sticky="w")

        self.create_label_combobox(
            top_frame, "Mode", ["MANUAL", "PLAYING"],
            command=self.mode_selected,
            value=self.configuracao.mode
        )[0].grid(row=0, column=0, padx=10, pady=5)

        self.create_label_combobox(
            top_frame, "Team color", ["Yellow", "Blue"],
            command=self.team_color_selected, value=team_color
        )[0].grid(row=0, column=1, padx=10, pady=5)

        self.create_label_combobox(
            top_frame, "Playing side", ["Left", "Right"],
            command=self.playing_side_selected, value=playing_side
        )[0].grid(row=0, column=2, padx=10, pady=5)

    def create_mappings_frame(self):
        mappings_frame = tk.Frame(self.root)
        mappings_frame.grid(row=1, column=0, columnspan=3, padx=10, pady=10, sticky="w")
        
        mappings = {
            "0": self.configuracao.sslvision_team_robot_id_mapping_0,
            "1": self.configuracao.sslvision_team_robot_id_mapping_1,
            "2": self.configuracao.sslvision_team_robot_id_mapping_2
        }

        for i, (key, value) in enumerate(mappings.items()):
            tk.Label(mappings_frame, text=f"Mapping {i+1}")\
                .grid(row=i, column=0, padx=10, pady=5)
            
            integer_var = tk.IntVar(value=value)

            entry = tk.Entry(mappings_frame, width=5, textvariable=integer_var)
            entry.grid(row=i, column=1, padx=5, pady=5)

            self.robot_mapping_id_entry_mapping[str(i)] = entry
            
            self\
                .create_combobox(mappings_frame, self.robot_ids, value=key)\
                .grid(row=i, column=2, padx=5, pady=5)

        tk.Button(mappings_frame, text="Submit mappings", command=self.submit_mappings)\
            .grid(row=3, column=1, columnspan=2, pady=10)

    def create_ids_frame(self):
        ids_frame = tk.Frame(self.root)
        ids_frame.grid(row=2, column=0, columnspan=3, padx=10, pady=5, sticky="w")

        created_frame, self.desired_keeper_id_combobox = self\
            .create_label_combobox(
                ids_frame,
                "Desired keeper id",
                self.robot_ids,
                value=str(self.configuracao.team_roles_goalkeeper_id)
            )
        
        created_frame.grid(row=0, column=0, padx=10, pady=5)
        
        created_frame, self.attacker_id_combobox = self\
            .create_label_combobox(
                ids_frame,
                "Attacker id",
                self.robot_ids,
                value=str(self.configuracao.team_roles_attacker_id
            ))
        
        created_frame.grid(row=0, column=1, padx=10, pady=5)
        
        created_frame, self.defensor_id_combobox = self\
            .create_label_combobox(
                ids_frame,
                "Defensor id",
                self.robot_ids,
                value=str(self.configuracao.team_roles_defensor_id)
            )
        
        created_frame.grid(row=0, column=2, padx=10, pady=5)

        tk.Button(ids_frame, text="Submit ids", command=self.submit_ids)\
            .grid(row=1, column=1, columnspan=2, pady=10)

        for i in range(0, 3):
            self\
                .create_robot_behavior_section(self.root, i)\
                .grid(row=3 + i + 1, column=0, columnspan=3, pady=5)

    def create_manual_frame(self):
        manual_frame = tk.Frame(self.root)
        manual_frame.grid(row=7, column=0, columnspan=3, padx=10, pady=5, sticky="w")

        created_frame, _ = self.create_label_combobox(
            manual_frame,
            "Robot id to send manual command",
            self.robot_ids
        )

        created_frame.grid(row=0, column=0, padx=10, pady=5)

        control_frame = tk.Frame(manual_frame)
        control_frame.grid(row=1, column=0, columnspan=3, pady=10)

        tk.Button(
            control_frame,
            text="Up",
            command=self.move_up
        ).grid(row=0, column=1, padx=5, pady=5)

        tk.Button(
            control_frame,
            text="Left",
            command=self.move_left
        ).grid(row=1, column=0, padx=5, pady=5)

        tk.Button(
            control_frame,
            text="Right",
            command=self.move_right
        ).grid(row=1, column=2, padx=5, pady=5)

        tk.Button(
            control_frame,
            text="Down",
            command=self.move_down
        ).grid(row=2, column=1, padx=5, pady=5)

    def create_replacement_frame(self):
        # replacement_frame = tk.Frame(self.root)
        # replacement_frame.grid(row=8, column=0, columnspan=3, padx=10, pady=10, sticky="w")

        # created_frame, _ =self.create_label_combobox(
        #     replacement_frame,
        #     "Replaced robot",
        #     self.robot_ids)
            
        # created_frame.grid(row=0, column=0, padx=10, pady=5)
        
        # tk.Button(replacement_frame, text="Submit", command=self.submit_replacement)\
        #     .grid(row=0, column=1, columnspan=2, pady=5)
        pass

    def create_label_combobox(self, root, label, options, value=None, command=None):
        frame = tk.Frame(self.root)
        tk.Label(frame, text=label).pack(side="left")
        combobox = ttk.Combobox(frame, values=options, state="readonly")
        if command:
            combobox.bind("<<ComboboxSelected>>", lambda e: command(combobox.get()))
        combobox.pack(side="left")

        if value is not None:
            combobox.set(value)

        return frame, combobox

    def create_combobox(self, root, options, value=None):
        combobox = ttk.Combobox(root, values=options, state="readonly", width=3)
        if value is not None:
            combobox.set(value)
        return combobox

    def create_robot_behavior_section(self, root, robot_id):
        frame = tk.Frame(self.root)
        tk.Label(frame, text=f"Robot {robot_id} behavior").grid(row=0, column=0)

        behavior_combobox = ttk.Combobox(
            frame,
            values=["STOPPED", "MANUAL", "BALL_TRACKER", "POSITIONING"],
            state="readonly"
        )

        behavior_combobox.grid(row=0, column=1, padx=5)

        self.robot_id_combobox_mapping[robot_id] = behavior_combobox

        tk.Label(frame, text="x").grid(row=0, column=2)
        tk.Entry(frame, width=5).grid(row=0, column=3)
        tk.Label(frame, text="y").grid(row=0, column=4)
        tk.Entry(frame, width=5).grid(row=0, column=5)
        tk.Button(
            frame,
            text="Submit",
            command=lambda: self.submit_robot_behavior(robot_id)
        ).grid(row=0, column=6, padx=5)

        return frame

    def mode_selected(self, value):
        self.configuracao.mode = value
        print(f"Mode selected: {value}")

    def team_color_selected(self, value):
        self.configuracao.team_is_yellow_team = value == "Yellow"
        print(f"Team color selected: {value}")

    def playing_side_selected(self, value):
        is_left = value == "Left"

        if is_left:
            self.configuracao.team_is_yellow_left_team = self.configuracao.team_is_yellow_team
        else:
            self.configuracao.team_is_yellow_left_team = not self.configuracao.team_is_yellow_team

        print(f"Playing side selected: {value}")

    def submit_mappings(self):
        robot_mapping_id_0 = int(self.robot_mapping_id_entry_mapping["0"].get())
        robot_mapping_id_1 = int(self.robot_mapping_id_entry_mapping["1"].get())
        robot_mapping_id_2 = int(self.robot_mapping_id_entry_mapping["2"].get())

        self.configuracao.sslvision_team_robot_id_mapping_0 = int(robot_mapping_id_0)
        self.configuracao.sslvision_team_robot_id_mapping_1 = int(robot_mapping_id_1)
        self.configuracao.sslvision_team_robot_id_mapping_2 = int(robot_mapping_id_2)

        print("Mappings submitted")

    def submit_ids(self):
        desired_keeper_id = int(self.desired_keeper_id_combobox.get())
        attacker_id = int(self.attacker_id_combobox.get())
        defensor_id = int(self.defensor_id_combobox.get())

        self.configuracao.team_roles_goalkeeper_id = desired_keeper_id
        self.configuracao.team_roles_attacker_id = attacker_id
        self.configuracao.team_roles_defensor_id = defensor_id

        print("Ids submitted")

    def submit_robot_behavior(self, robot_id):
        combox = self.robot_id_combobox_mapping.get(robot_id)

        if combox:
            behavior = combox.get()
            robot_behavior_enum =\
                EnumUtils.get_robot_behavior_enum_by_name(behavior)
            
            if robot_id == 0:
                self.configuracao.runtime_behavior_robot_0 = robot_behavior_enum
            elif robot_id == 1:
                self.configuracao.runtime_behavior_robot_1 = robot_behavior_enum
            elif robot_id == 2:
                self.configuracao.runtime_behavior_robot_2 = robot_behavior_enum

            print(f"Behavior for Robot {robot_id} submitted")

    def move_up(self):
        self.configuracao.runtime_manual_command = ManualCommandEnum.UP
        print("Move up command sent")

    def move_down(self):
        self.configuracao.runtime_manual_command = ManualCommandEnum.DOWN
        print("Move down command sent")

    def move_left(self):
        self.configuracao.runtime_manual_command = ManualCommandEnum.LEFT
        print("Move left command sent")

    def move_right(self):
        self.configuracao.runtime_manual_command = ManualCommandEnum.RIGHT
        print("Move right command sent")

    def submit_replacement(self):
        print("Not implemented")

if __name__ == "__main__":
    root = tk.Tk()
    app = ControlCenter(root)
    root.mainloop()