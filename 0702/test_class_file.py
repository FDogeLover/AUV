class FileHandler:
    def __init__(self, file_name):
        self.file_name = file_name
        self.file = None  # 文件对象初始为None
        self.dict_file_info = {}

    def open_write(self):
        self.file = open(self.file_name, 'w')
        print("file open_write success")

    def write_data(self, str_position: str, index_pos: int):
        line = f"{str_position}, {index_pos}\n"
        if self.file is not None:
            self.file.write(line)

    def open_add(self):
        self.file = open(self.file_name, 'a')
        print("file open_write success")

    def write_open(self, str_position: str, index_pos: int):
        self.open_add()
        self.write_data(str_position, index_pos)
        self.file_close()

    def open_read(self):
        self.file = open(self.file_name, 'r')
        print("file open_read success")

    def read_data(self):
        if self.file is not None:
            for line in self.file:
                # 去除每行末尾的换行符
                line = line.strip()
                # 使用逗号分隔符拆分字符串，得到字符串和整数部分
                parts = line.split(', ')
                # 确保我们有两部分数据
                if len(parts) == 2:
                    self.dict_file_info[int(parts[1])] = parts[0]
                else:
                    print(f"Invalid line format: {line}")
        else:
            print("file has not inited")

    def print_dict_info(self):
        if len(self.dict_file_info) > 0:
            print(self.dict_file_info)

    def return_dict_info(self):
        if len(self.dict_file_info) > 0:
            return self.dict_file_info

    def file_close(self):
        if self.file is not None:
            self.file.close()
            print("文件已关闭")

    def __del__(self):
        self.file_close()