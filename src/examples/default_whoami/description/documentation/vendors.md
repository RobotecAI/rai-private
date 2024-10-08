# Vendors Initialization Examples

## Ollama

For installation see: https://ollama.com/

```python
from langchain_community.chat_models import ChatOllama

llm = ChatOllama(model='llava')
```

## OpenAI

```bash
export OPENAI_API_KEY="sk-..."
```

```python
from langchain_openai.chat_models import ChatOpenAI

llm = ChatOpenAI(
    model="gpt-4o",
)
```

## AWS Bedrock

```bash
export AWS_ACCESS_KEY_ID="..."
export AWS_SECRET_ACCESS_KEY="..."
export AWS_SESSION_TOKEN="..."
```

```python
from langchain_aws.chat_models import ChatBedrock

llm = ChatBedrock(
    model="anthropic.claude-3-opus-20240229-v1:0",
)
```

# Caching

## Redis

`ScenarioRunner` supports Redis cache through langchain. Make sure to set

```bash
export REDIS_CACHE_HOST="redis://<host>"
```

Self hosting Redis:

```bash
docker run -p 6379:6379 -d redis:latest
export REDIS_CACHE_HOST="redis://localhost:6379"
```

For more invormation see [redis.io/docs/latest/operate/oss_and_stack/install/install-stack/docker/](https://redis.io/docs/latest/operate/oss_and_stack/install/install-stack/docker/)
