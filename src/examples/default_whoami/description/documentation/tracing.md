# Tracing Configuration

RAI supports tracing capabilities to help monitor and analyze the performance of your AI models, at a minor performance cost. By default, tracing is off. This document outlines how to configure tracing for your RAI project.

## Configuration

Tracing configuration is managed through the `config.toml` file. The relevant parameters for tracing are:

### Project Name

The `project` field under the `[tracing]` section sets the name for your tracing project. This name will be used to identify your project in the tracing tools.

> [!NOTE]  
> Project name is currently only used by LangSmith. Langfuse will upload traces to the default project.

### Langfuse (open-source)

[Langfuse](https://langfuse.com/) is an open-source observability & analytics platform for LLM applications.

To enable Langfuse tracing:

1. Set `use_langfuse = true` in the `config.toml` file.
2. Set the `LANGFUSE_PUBLIC_KEY` and `LANGFUSE_SECRET_KEY` environment variables with your Langfuse credentials.
3. Optionally, you can specify a custom Langfuse host by modifying the `host` field under `[tracing.langfuse]`.

### LangSmith (closed-source, paid)

[LangSmith](https://www.langchain.com/langsmith) is a platform for debugging, testing, and monitoring LangChain applications.

To enable LangSmith tracing:

1. Set `use_langsmith = true` in the `config.toml` file.
2. Set the `LANGCHAIN_API_KEY` environment variable with your LangSmith API key.

## Usage

To enable tracing in your RAI application, you need to import the get_tracing_callbacks() function and add it to the configuration when invoking your agent or model. Here's how to do it:

1. First, import the get_tracing_callbacks() function:

```python
from rai.utils.model_initialization import get_tracing_callbacks
```

2. Then, add it to the configuration when invoking your agent or model:

```python
response = agent.invoke(
    input_dict,
    config={"callbacks": get_tracing_callbacks()}
)
```

By adding the get_tracing_callbacks() to the config parameter, you enable tracing for that specific invocation. The get_tracing_callbacks() function returns a list of callback handlers based on your configuration in config.toml.

## Troubleshooting

If you encounter issues with tracing:

1. Ensure all required environment variables are set correctly.
2. Check whether tracing is on by checking whether `use_langsmith` or `use_langfuse` flag is set to `true` in `config.toml`.
3. Verify that you have the necessary permissions and valid API keys for the tracing services you're using.
4. Look for any error messages in your application logs related to tracing initialization.

For more detailed information on using these tracing tools, refer to their respective documentation:

- [LangSmith Documentation](https://docs.langchain.com/docs/langsmith)
- [Langfuse Documentation](https://langfuse.com/docs)
