import re
import json
filepath1="/home/autolabor/voice_jieya/separate/word.txt"
filepath2="/home/autolabor/voice_jieya/separate/input.txt"
filepath3="/home/autolabor/voice_jieya/separate/output.txt"
with open(filepath1, 'r', encoding='utf-8') as file:
    content = file.read()
#print(content)
#word_dic=eval(content)
#word_list =list(word_dic.keys())
#print(word_list)
word_list = content.split()
#print(word_list)
max_length=len(sorted(word_list, key=lambda x: len(x))[-1])
with open(filepath2, 'r', encoding='utf-8') as file:
    content2 = file.read()
chinese_characters = re.findall('[\u4e00-\u9fa5]', content2)
sentence = ''.join(chinese_characters)
ans_forward=[]  #存放正向匹配的结果
ans_reverse=[]  #存放逆向匹配的结果

len_row=len(sentence)  #len_orw为当前为划分句子的长度
while len_row>0:       #当前待划分句子长度为0时，结束划分
    divide = sentence[0:max_length]    #从前向后截取长度为max_length的字符串
    while divide not in word_list:      #当前截取的字符串不在分词字典中，则进循环
        if len(divide)==1:             #当前截取的字符串长度为1时，说明分词字典无匹配项
            break                      #直接保留当前的一个字
        divide=divide[0:len(divide)-1] #当前截取的字符串长度减一
    if len(divide) != 1:
        ans_forward.append(word_list.index(divide))         #记录下当前截取的字符串
    sentence = sentence[len(divide):]  #截取未分词的句子
    len_row = len(sentence)            #记录未分词的句子的长度

sentence = ''.join(chinese_characters)
len_row=len(sentence)
while len_row>0 :
    divide = sentence[-max_length:]
    while divide not in word_list:
        if len(divide)==1:
            break
        divide=divide[1:]   #注意这里缩短截取字符串是缩短前部分，保留后部分
    ans_reverse.append(divide)
    sentence = sentence[0:len(sentence)-len(divide)]
    len_row = len(sentence)
ans_reverse=ans_reverse[::-1]

# 打开文件并写入列表内容
with open(filepath3, 'w', encoding='utf-8') as file:
    for item in ans_forward:
        file.write(str(item) + '\n')

