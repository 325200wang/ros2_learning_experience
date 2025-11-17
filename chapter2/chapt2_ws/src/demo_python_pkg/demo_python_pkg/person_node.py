class PersonNode:#类内部方法第一个参数都是self,代表其本身
    def __init__(self, name: str, age: int ) -> None:
        print("PersonNode's __init__ func is quoted")
        self.age = age
        self.name = name

    def eat(self, food_name: str):
        print(f'my name is {self.name}, I am {self.age} old, and I am eating {food_name}')
    
    def main():
        node = PersonNode('Floating', 22)
        node.eat('牛排')