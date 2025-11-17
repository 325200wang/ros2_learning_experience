from demo_python_pkg.person_node import PersonNode

class WriterNode(PersonNode):#表示继承PersonNode
    def __init__(self, name: str, age: int, book: str) -> None:
        super().__init__(name, age)
        print('WriterNode 的 __init__ func 被调用')
        self.book = book#添加book属性
def main():
    node = WriterNode('Solivagant')#实例化WriterNode对象
    node.eat('KFC')
