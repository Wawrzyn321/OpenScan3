from pydantic import BaseModel, Field


class NodeMCUConfig(BaseModel):
    url: str = Field(description="NodeMCU base URL, e.g. 'http://192.168.1.50'")
    timeout: float = Field(5.0, description="HTTP request timeout in seconds.")
    poll_interval: float = Field(0.05, description="Seconds between /motor/status polls when waiting for movement to finish.")
