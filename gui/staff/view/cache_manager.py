class CacheManager:
    def __init__(self):
        self.cart = []

    def add_item(self, item):
        self.cart.append(item)
        print(f"장바구니에 추가됨: {item}")

    def remove_item(self, item):
        if item in self.cart:
            self.cart.remove(item)
            print(f"장바구니에서 삭제됨: {item}")
        else:
            print(f"장바구니에 항목이 없습니다: {item}")

    def get_items(self):
        return self.cart

    def clear_items(self):
        self.cart = []
