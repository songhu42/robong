import os

from ros2env.verb import VerbExtension

# python 설계상 프로그램에서 외부에서 정의된 os.env 바뀔 수 없다. 
class SetVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument('env_name', help='Name of env')
        parser.add_argument('value', help='value of env')

    def main(self, *, args):
        if args.env_name or args.value:
            print('test')
            print(args.env_name)
            print(args.value)
            os.environ[args.env_name] = args.value 
            print(os.getenv(args.env_name))