from stable_baselines3 import DDPG
import os

class ModelHelper():
    __models_folder_name = "models"
    __attacker_folder_name = "attacker_v2"
    __goalkeeper_folder_name = "goalkeeper"
    __default_model_name = "DDPG_model"
    __latest_attacker_datetime_folder_name = "2024_5_19_16_19_43"
    __latest_goalkeeper_datetime_folder_name = "2024_5_6_16_34_4"

    @staticmethod
    def _get_relative_path_to_model_folder(role_name: str):
        current_directory_path = os.path.abspath(os.path.dirname(__file__))
        return os.path.join(
            current_directory_path,
            "..",
            "..",
            ModelHelper.__models_folder_name,
            role_name)

    @staticmethod
    def get_attacker_model():
        return ModelHelper.get_model_by_relative_path(
            ModelHelper.__attacker_folder_name,
            ModelHelper.__latest_attacker_datetime_folder_name)
    
    @staticmethod
    def get_goalkeeper_model():
        return ModelHelper.get_model_by_relative_path(
            ModelHelper.__goalkeeper_folder_name,
            ModelHelper.__latest_goalkeeper_datetime_folder_name)
    
    def get_model_by_relative_path(
        role_name: str,
        datetime_folder_name: str,
        model_name: str | None = None
    ):
        if model_name is None:
            model_name = ModelHelper.__default_model_name

        model_path = os.path.join(
            ModelHelper._get_relative_path_to_model_folder(role_name),
            datetime_folder_name,
            model_name)

        return DDPG.load(model_path)