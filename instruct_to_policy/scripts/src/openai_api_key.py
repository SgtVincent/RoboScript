import openai
openai.api_base = "https://api.openai-sb.com/v1"

# OPENAI_SB_API_KEY = 'sb-95cb9872616b7d00c8e7bb26e68b08417e3b8a3070db6a8a'
OPENAI_SB_API_KEY = 'sk-Dr1KHY31ZFGHGvLhD0927d94Fc58483f9c5d0c9fDd5800C2'
OPENAI_API_KEY = OPENAI_SB_API_KEY
    
openai.api_base = 'https://api.pumpkinaigc.online/v1'

def test_openai_sb():
    
    openai.api_key = OPENAI_SB_API_KEY
    openai.api_base = 'https://api.pumpkinaigc.online/v1'
    
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