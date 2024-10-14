import tkinter

# 키 코드 입력 변수 선언
key = 0
# 키를 눌렀을 때 실행할 함수 정의
def key_down(e):
    global key # key을 전역 변수로 취급
    key = e.keycode
    print(f"KEY: {key}")
    
root = tkinter.Tk()
root.title("키 코드 얻기")
root.bind("<KeyPress>", key_down)
root.mainloop()
