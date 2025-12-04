import xml.etree.ElementTree as ET
import os

# 获取URDF文件路径
urdf_path = os.path.join('urdf', 'arm3dof.urdf')

try:
    # 解析URDF文件
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    print("✅ URDF文件格式正确!")
    
    # 检查必要的元素
    links = root.findall('link')
    joints = root.findall('joint')
    
    print(f"📋 连杆数量: {len(links)}")
    print(f"📋 关节数量: {len(joints)}")
    
    # 检查每个连杆是否有必要的元素
    for link in links:
        link_name = link.get('name')
        visual = link.find('visual')
        collision = link.find('collision')
        inertial = link.find('inertial')
        
        print(f"\n🔗 连杆: {link_name}")
        print(f"   - 可视化: {'✅' if visual is not None else '❌'}")
        print(f"   - 碰撞: {'✅' if collision is not None else '❌'}")
        print(f"   - 惯性: {'✅' if inertial is not None else '❌'}")
        
    print("\n✅ 所有检查通过!")
    
except ET.ParseError as e:
    print(f"❌ URDF文件格式错误: {e}")
except FileNotFoundError:
    print(f"❌ 找不到URDF文件: {urdf_path}")
except Exception as e:
    print(f"❌ 检查过程中出现错误: {e}")
