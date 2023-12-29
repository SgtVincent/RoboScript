import openai
import requests
import time
import cv2
import os


class SMMS(object):
    def __init__(self):
        self.headers = {'Authorization': 'stsdBZgf0WBeJoYfiND1iKVUW6zuGHS5'}

    def isDiskFull(self):
        url = 'https://sm.ms/api/v2/profile'
        res = requests.post(url, headers=self.headers, timeout=5).json()
        if res["disk_usage_raw"] > 0.9 * res["disk_limit_raw"]:
            return True

    def upload(self, filepath):
        files = {'smfile': open(filepath, 'rb')}
        url = 'https://sm.ms/api/v2/upload'
        res = requests.post(url, files=files, headers=self.headers, timeout=15).json()
        # print(json.dumps(res, indent=4))
        # print(res)
        return res['data']['url']

    def getHistory(self):
        url = 'https://sm.ms/api/v2/upload_history'
        res = requests.get(url, headers=self.headers, timeout=5).json()
        return res

    def deleteHistory(self):
        history = self.getHistory()
        lastTime = time.time()
        for item in history["data"]:
                url = 'https://sm.ms/api/v2/delete/{}'.format(item["hash"])
                res = requests.get(url, headers=self.headers, timeout=5).json()
        print("------------------------------------------------------History deleted!----------------------------------------")
        

def call_gpt4_api(prompt,image_url):
    # 需要先设置你的 OpenAI API 密钥
    openai.api_key = 'sk-Dr1KHY31ZFGHGvLhD0927d94Fc58483f9c5d0c9fDd5800C2'
    
    openai.api_base = 'https://api.pumpkinaigc.online/v1'
    
    # 调用模型
    response = openai.ChatCompletion.create(
        model="gpt-4-vision-preview",
        messages=[
        {
            "role": "user",
                "content": 
                [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": 
                            {
                                "url": image_url
                            },
                    },
                ],
        }
        ],
        max_tokens=3000,
    )

    # 返回模型的文本输出
    return response.choices[0].message['content']


def gpt_check(image,task_query, Num_check_time=5):
    
    current_path = os.getcwd()
    image_path = os.path.join(current_path, f'src/env/test_image/{str(int(time.time()))}.jpg')
    prompt_path = os.path.join(current_path, 'src/env/prompt.txt')

    cv2.imwrite(image_path,image)

    url = SMMS()
    url.deleteHistory()
    image_url = url.upload(image_path)
    print('------------------------------------image_url---------------------------------------')
    print(image_url)
    with open(prompt_path, 'r', encoding='utf-8') as file:
        prompt = file.read()
    # prompt = "Our task is to put the drink can into the bowl. The picture below is the final picture. Was the mission successful? Please analyze this scenario first (do not output the analysis process), and then directly answer whether the task was successful (yes or no).  Note: Only output 'yes' or 'no', do not output anything else"
    # print(prompt)
    prompt = 'Our task is' + task_query + '. ' + prompt
    check_state = 'task'
    i = 0
    while ('yes' not in check_state and 'no' not in check_state) and i < Num_check_time:
        result = call_gpt4_api(prompt,image_url)
        print('------------------------------------result---------------------------------------')
        # print(result)


        result_dict = {}
        lines = result.strip().split('\n')
        for line in lines:
            if ':' in line:  # 确保行中含有冒号
                # 分割 key 和 value
                key, value = line.split(':', 1)
                # 清理 key 和 value 并添加到字典中
                result_dict[key.strip().strip("'").strip('"').strip()] = value.strip().strip(';').strip(',').strip('"')

        print(result_dict)
        check_state = result_dict['mission success'].lower()
        i = i + 1

    judge = 'yes' in check_state

    return judge, result_dict['your_explanation']


if __name__ == '__main__': 
    image = cv2.imread('/home/cuite/data/test_code/2.jpg')
    check_state, explanation = gpt_check(image,'')
    print('------------------------------------output_state---------------------------------------')
    print(check_state)
    print('------------------------------------output_explanation---------------------------------------')
    print(explanation)