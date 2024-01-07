from transformers import (
    AutoModelForCausalLM,
    AutoTokenizer
)
import torch


class RoveAI():
    def __init__(self):
        self.base_model = "microsoft/phi-2"
        self.model = AutoModelForCausalLM.from_pretrained(
            "microsoft/phi-2", torch_dtype=torch.float32, device_map="cpu",
            trust_remote_code=True)
        self.tokenizer = AutoTokenizer.from_pretrained(self.base_model)

    def predict(self, message, history):
        inputs = self.tokenizer.encode(message, return_tensors="pt")
        outputs = self.model.generate(
            inputs, max_length=1000, do_sample=True, top_k=50)
        return self.tokenizer.decode(outputs[0], skip_special_tokens=True)
