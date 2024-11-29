import openai
# openai.api_base = "PLEASE_ENTER_API_BASE_URL"
OPENAI_API_KEY = "Please enter your OpenAI API key here"

def test_openai_sb():
    
    openai.api_key = OPENAI_API_KEY
    
    messages =  [
        {"role": "system", "content": f"You are a friendly assistant. You are helping a user to do some tasks."},
        {"role": "user", "content": "Good day!"},
    ]
    
    response = openai.ChatCompletion.create(
        messages=messages,
        temperature=0.5,
        model='gpt-3.5-turbo-0613',
    )
    
    print(response)
    
if __name__ == "__main__":
    test_openai_sb()