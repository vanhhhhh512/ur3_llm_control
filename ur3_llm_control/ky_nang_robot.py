"""Ba robot skill cong khai: home, pick, place.

Day la tang duy nhat duoc phep dieu khien robot. LLM khong bao gio goi thang
vao MoveIt: no chi chon ten skill va tham so, con moi quy dao deu do cac ham
o day dung nen roi giao cho MoveIt lap ke hoach co tranh va cham.

Moi skill tra ve mot ma trang thai trong danh_muc (SUCCESS, FAILED,
INVALID_OBJECT, INVALID_ZONE, PLANNING_FAILED) kem mot dong giai thich.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

from geometry_msgs.msg import Pose

from ur3_llm_control.danh_muc import (
    LAP_KE_HOACH_THAT_BAI,
    THANH_CONG,
    THAT_BAI,
    VAT_THE_KHONG_HOP_LE,
    VUNG_KHONG_HOP_LE,
)
from ur3_llm_control.giao_tiep_moveit import GiaoTiepMoveIt, LoiMoveIt, tao_pose
from ur3_llm_control.mo_hinh_workcell import Workcell

# Cac link cua co tay duoc phep cham vao vat dang cam, neu khong MoveIt se
# bao va cham ngay khi vat vua duoc gan vao tool0.
LINK_DUOC_CHAM = ("tool0", "wrist_3_link", "wrist_2_link", "flange")


@dataclass
class KetQuaKyNang:
    trang_thai: str
    thong_diep: str = ""

    @property
    def thanh_cong(self) -> bool:
        return self.trang_thai == THANH_CONG


class KyNangRobot:
    def __init__(self, moveit: GiaoTiepMoveIt, workcell: Workcell,
                 ghi_log: Optional[Callable[[str], None]] = None,
                 dong_bo_gazebo: Optional[Callable[[str, Sequence[float]], None]] = None,
                 bam_theo_tay: Optional[Callable[[Optional[str]], None]] = None) -> None:
        self._moveit = moveit
        self._wc = workcell
        self._log = ghi_log or (lambda _: None)
        self._dong_bo = dong_bo_gazebo          # dat lai vi tri vat trong Gazebo
        self._bam_theo = bam_theo_tay           # bat/tat che do vat bam theo tool0

    # ------------------------------------------------------------ tien ich
    def _tu_the(self, xyz: Sequence[float]) -> Pose:
        return tao_pose(xyz, self._wc.chuyen_dong.quaternion_gap)

    def _di_thang(self, cac_diem: List[Sequence[float]], nhan: str,
                  tranh_va_cham: bool = True) -> None:
        """Noi cac diem bang duong thang.

        Hai doan sat luc gap va luc tha chay voi tranh_va_cham = False. Day la
        cach lam thong thuong cua pick-and-place trong MoveIt: khi khoi da gan
        vao tool0 thi no van dang cham mat ban, neu bat kiem va cham thi moi
        duong nang len deu bi coi la va cham va tra ve 0%. Doan nay chi dai
        vai centimet theo phuong thang dung trong vung da biet chac la trong,
        va van bi rang buoc boi gioi han khop cua robot.
        """
        cd = self._wc.chuyen_dong
        ty_le = self._moveit.di_theo_duong_thang(
            [self._tu_the(diem) for diem in cac_diem],
            cd.buoc_cartesian, cd.ty_le_cartesian_toi_thieu,
            tranh_va_cham=tranh_va_cham,
            he_so_van_toc=cd.he_so_van_toc, he_so_gia_toc=cd.he_so_gia_toc)
        self._log(f"    {nhan}: cartesian {ty_le * 100:.0f}%"
                  + ("" if tranh_va_cham else " (doan tiep can, bo kiem va cham)"))

    def _di_khop(self, khop: Sequence[float], nhan: str) -> None:
        cd = self._wc.chuyen_dong
        self._moveit.di_toi_khop(khop, cd.he_so_van_toc, cd.he_so_gia_toc)
        self._log(f"    {nhan}: done")

    def _di_toi_diem(self, xyz: Sequence[float], nhan: str) -> None:
        """Di toi mot diem bang cach lap ke hoach theo rang buoc tu the.

        Khong goi IK truoc: de MoveIt tu chon nhanh nghiem khop, nho vay tranh
        duoc truong hop IK tra ve mot tu the ky di ma OMPL khong noi toi duoc.
        """
        cd = self._wc.chuyen_dong
        try:
            self._moveit.di_toi_diem(self._tu_the(xyz), cd.he_so_van_toc, cd.he_so_gia_toc)
        except LoiMoveIt as loi:
            # Khong di thang duoc thi ghe qua tu the trung chuyen roi thu lai.
            # Tu vi tri gap cua vat nay sang vung dat kia, duong noi truc tiep
            # co the khong ton tai vi tay may phai luon qua cac vat da dat.
            self._log(f"    {nhan}: blocked, routing via transit pose ({loi})")
            self._moveit.di_toi_khop(cd.tu_the_trung_chuyen,
                                     cd.he_so_van_toc, cd.he_so_gia_toc)
            self._moveit.di_toi_diem(self._tu_the(xyz), cd.he_so_van_toc, cd.he_so_gia_toc)
        self._log(f"    {nhan}: done")

    def _len_cao_an_toan(self) -> None:
        """Nang tool0 len do cao mang vat truoc khi di ngang."""
        cd = self._wc.chuyen_dong
        z_an_toan = self._wc.cao_mat_ban + self._wc.canh_vat + cd.cao_di_chuyen
        # Dung chinh vi tri hien tai theo phuong ngang, chi doi z
        # (lay tu FK cua MoveIt qua diem cuoi da biet nen khong can goi them).
        self._log(f"    nang len z = {z_an_toan:.3f}")

    # --------------------------------------------------------------- home
    def home(self) -> KetQuaKyNang:
        if self._wc.dang_cam is not None:
            return KetQuaKyNang(THAT_BAI, f"tay may con dang cam {self._wc.dang_cam}")
        try:
            self._di_khop(self._wc.chuyen_dong.tu_the_home, "go home")
        except LoiMoveIt as loi:
            return KetQuaKyNang(LAP_KE_HOACH_THAT_BAI, str(loi))
        return KetQuaKyNang(THANH_CONG)

    # --------------------------------------------------------------- pick
    def pick(self, ten_vat: str) -> KetQuaKyNang:
        if not self._wc.co_vat(ten_vat):
            return KetQuaKyNang(VAT_THE_KHONG_HOP_LE, f"{ten_vat} khong co trong workcell")
        if self._wc.dang_cam is not None:
            return KetQuaKyNang(THAT_BAI, f"tay may dang cam {self._wc.dang_cam}")

        cd = self._wc.chuyen_dong
        diem_gap = self._wc.diem_gap(ten_vat)
        treo = [diem_gap[0], diem_gap[1], diem_gap[2] + cd.cao_tiep_can]
        mang = [diem_gap[0], diem_gap[1], diem_gap[2] + cd.cao_di_chuyen]

        lech_z = -(self._wc.canh_vat / 2.0 + self._wc.chuyen_dong.khe_ho_gap)
        try:
            self._di_toi_diem(treo, f"approach above {ten_vat}")
            # Xoa vat can truoc khi ha xuong: tool0 se nam ngay tren mat khoi
            self._moveit.xoa_hop(ten_vat)
            self._di_toi_diem(diem_gap, "descend to grasp")

            # Dong "gripper hut": gan khoi vao tool0 va cho no bam theo tay
            # may trong Gazebo.
            self._moveit.gan_vat(ten_vat, self._wc.canh_vat, lech_z, LINK_DUOC_CHAM)
            if self._bam_theo is not None:
                self._bam_theo(ten_vat)
            self._wc.ghi_nhan_gap(ten_vat)
            self._log(f"    attached {ten_vat} to {self._wc.link_cong_tac}")

            self._di_toi_diem(mang, "lift object")
        except LoiMoveIt as loi:
            if self._bam_theo is not None:
                self._bam_theo(None)
            return KetQuaKyNang(LAP_KE_HOACH_THAT_BAI, str(loi))
        return KetQuaKyNang(THANH_CONG)

    # -------------------------------------------------------------- place
    def place(self, ten_vat: str, ten_vung: str) -> KetQuaKyNang:
        if not self._wc.co_vat(ten_vat):
            return KetQuaKyNang(VAT_THE_KHONG_HOP_LE, f"{ten_vat} khong co trong workcell")
        if not self._wc.co_vung(ten_vung):
            return KetQuaKyNang(VUNG_KHONG_HOP_LE, f"{ten_vung} khong co trong workcell")
        if self._wc.dang_cam != ten_vat:
            return KetQuaKyNang(THAT_BAI,
                                f"tay may dang cam {self._wc.dang_cam or 'khong gi ca'}, "
                                f"khong phai {ten_vat}")
        chu_cu = self._wc.vung_bi_chiem(ten_vung)
        if chu_cu is not None and chu_cu != ten_vat:
            return KetQuaKyNang(THAT_BAI, f"{ten_vung} da bi {chu_cu} chiem")

        cd = self._wc.chuyen_dong
        diem_tha = self._wc.diem_tha(ten_vung)
        treo = [diem_tha[0], diem_tha[1], diem_tha[2] + cd.cao_tiep_can]
        mang = [diem_tha[0], diem_tha[1], diem_tha[2] + cd.cao_di_chuyen]

        try:
            self._di_toi_diem(mang, f"carry {ten_vat} to {ten_vung}")
            self._di_toi_diem(diem_tha, "descend to release")

            # Mo "gripper hut": go khoi khoi tool0 roi dat lai vao the gioi
            self._moveit.tha_vat(ten_vat)
            if self._bam_theo is not None:
                self._bam_theo(None)
            self._wc.ghi_nhan_tha(ten_vat, ten_vung)
            if self._dong_bo is not None:
                self._dong_bo(ten_vat, self._wc.vi_tri_vat[ten_vat])
            self._log(f"    released {ten_vat} at {ten_vung}")

            self._di_toi_diem(treo, "retreat")
            # Chi dua vat can tro lai the gioi SAU khi tay may da rut len,
            # neu khong tu the hien tai bi coi la dang va cham voi chinh no.
            self._moveit.them_hop(ten_vat, self._wc.vi_tri_vat[ten_vat],
                                  (self._wc.canh_vat,) * 3)
        except LoiMoveIt as loi:
            return KetQuaKyNang(LAP_KE_HOACH_THAT_BAI, str(loi))
        return KetQuaKyNang(THANH_CONG)

