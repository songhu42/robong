#include <stdio.h>

int main(void){
    int num1=0,num2=0;
    char oper = '\0';
    int result = 0;

    printf("수식을 입력하세요 : ");
    scanf("%d%c%d", &num1, &oper, &num2);

    if(oper == '+'){   // oper이 더하기이면
        result = num1+num2;
    }
    else if(oper == '-'){  // oper이 빼기이면
        result = num1-num2;
    }
    else if(oper == '*'){  // oper이 곱하기이면
        result = num1*num2;
    }
    else if(oper == '/'){  // oper이 나누기이면
        result = num1/num2;
    }
    
    printf("%d %c %d = %d\n", num1, oper, num2, result);

    return 0;

}