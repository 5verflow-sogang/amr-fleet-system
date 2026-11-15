"""Nav2 식당 프로필(footprint·순항 속도·장애물 높이)과 혼잡도 costmap 필터 오버레이 (nav2.launch.py nav2_profile·congestion_filter 인자).

nav2_params.yaml 전체를 복사하지 않고 바뀌는 키만 돌려준다 - nav2.launch.py가 controller_server의
FollowPath(DWA) override와 같은 방식으로, 로봇별 임시 yaml에 이 값만 더 써서 RewrittenYaml 뒤에
추가 parameters 소스로 얹는다 (ROS2 Node parameters는 여러 소스를 나중 것이 이기는 순서로 합친다).
기본(warehouse·none)은 아무 것도 덮어쓰지 않아 지금 물류센터 동작과 완전히 같다. ROS 없이 돈다.
"""

PROFILES = ('warehouse', 'restaurant')

# 서빙 로봇(Pudu BellaBot 크기, #299) 바닥 0.565 x 0.537 m -> 중심 기준 반길이(x)·반폭(y).
# 지금 footprint(0.6 x 0.4 m, nav2_params.yaml 0.3/0.2)와 같은 사각형 형식.
_RESTAURANT_HALF_LENGTH = 0.565 / 2
_RESTAURANT_HALF_WIDTH = 0.537 / 2
_RESTAURANT_FOOTPRINT = (
    f'[[{_RESTAURANT_HALF_LENGTH}, {_RESTAURANT_HALF_WIDTH}], '
    f'[{_RESTAURANT_HALF_LENGTH}, -{_RESTAURANT_HALF_WIDTH}], '
    f'[-{_RESTAURANT_HALF_LENGTH}, -{_RESTAURANT_HALF_WIDTH}], '
    f'[-{_RESTAURANT_HALF_LENGTH}, {_RESTAURANT_HALF_WIDTH}]]'
)
# 식당 순항 속도 [m/s]. 물류센터 DWA max_v(0.5)는 Nav KPI 기준 속도라 그대로 두고 식당만 올린다.
# 사람이 가까우면 안전 노드가 따로 묶는다(amr_safety config/safety.yaml warning_speed_limit)
_RESTAURANT_MAX_V = 0.8


def profile_overrides(nav2_params: dict, profile: str) -> dict:
    """{'global_costmap': {...}, 'local_costmap': {...}, 'DWA': {...}}에 덮어쓸 키만. warehouse면 빈 dict.

    nav2_params는 지금 쓰이지 않지만(식당 footprint는 물류센터 값과 무관한 고정값이다) 다른
    오버레이 함수(congestion_filter_overrides)와 시그니처를 맞추고, 나중에 기존 값 기준으로
    덧셈/비율 계산이 필요해지면 바로 쓸 수 있게 받아 둔다.
    """
    if profile not in PROFILES:
        raise ValueError(f'nav2_profile은 {", ".join(PROFILES)} 중 하나: {profile!r}')
    if profile == 'warehouse':
        return {}
    del nav2_params
    return {
        'global_costmap': {
            'footprint': _RESTAURANT_FOOTPRINT,
            # 바닥에서 0.38 m 높이의 LiDAR 관측을 식당에서만 장애물로 반영한다.
            'obstacle_layer': {'scan': {'max_obstacle_height': 2.0}},
        },
        'local_costmap': {
            'footprint': _RESTAURANT_FOOTPRINT,
            # 바닥에서 0.38 m 높이의 LiDAR 관측을 식당에서만 장애물로 반영한다.
            'obstacle_layer': {'scan': {'max_obstacle_height': 2.0}},
        },
        'DWA': {
            'footprint_half_length': _RESTAURANT_HALF_LENGTH,
            'footprint_half_width': _RESTAURANT_HALF_WIDTH,
            'max_v': _RESTAURANT_MAX_V,
        },
    }


CONGESTION_FILTERS = ('none', 'keepout', 'speed')

# 민지 혼잡도 노드(#296)가 내는 필터 정보 토픽 - 절대 이름이라 로봇 네임스페이스와 무관하게 같은
# 값을 그대로 쓴다(#296 PR 리뷰 답). 마스크 자체(/fleet/congestion_mask)는 info 메시지의
# filter_mask_topic 필드로 알려주므로 여기서 따로 설정할 게 없다.
_KEEPOUT_INFO_TOPIC = '/fleet/congestion_keepout_info'
_SPEED_INFO_TOPIC = '/fleet/congestion_speed_info'
_BASE_GLOBAL_PLUGINS = ['static_layer', 'obstacle_layer', 'inflation_layer']


def congestion_filter_overrides(filter_choice: str, speed_limit_topic: str = '') -> dict:
    """global_costmap에 덧붙일 costmap_filters 플러그인(plugins 리스트 전체 + 플러그인 블록).

    plugins는 리스트라 항목만 더하는 병합이 안 되므로(YAML 오버레이는 키 단위) 전체 목록을
    새로 준다 - 그래도 바뀌는 건 이 키 하나뿐이다. none이면 빈 dict(물류센터 그대로).

    speed_limit_topic: SpeedFilter가 속도 제한을 낼 절대 토픽. 비우면 SpeedFilter 기본값
    ("speed_limit", costmap 자신의 노드 네임스페이스 기준 - 즉 <ns>/global_costmap/speed_limit)을
    쓰는데, controller_server가 구독하는 기본 토픽은 <ns>/speed_limit라 서로 안 만난다(실측
    확인 - SpeedFilter 쪽 publisher count 0). nav2.launch.py가 로봇 네임스페이스로 절대 경로를
    만들어 넘겨준다.
    """
    if filter_choice not in CONGESTION_FILTERS:
        raise ValueError(f'congestion_filter는 {", ".join(CONGESTION_FILTERS)} 중 하나: {filter_choice!r}')
    if filter_choice == 'none':
        return {}
    if filter_choice == 'keepout':
        return {
            'plugins': _BASE_GLOBAL_PLUGINS + ['keepout_filter'],
            'keepout_filter': {
                'plugin': 'nav2_costmap_2d::KeepoutFilter',
                'enabled': True,
                'filter_info_topic': _KEEPOUT_INFO_TOPIC,
            },
        }
    speed_filter = {
        'plugin': 'nav2_costmap_2d::SpeedFilter',
        'enabled': True,
        'filter_info_topic': _SPEED_INFO_TOPIC,
    }
    if speed_limit_topic:
        speed_filter['speed_limit_topic'] = speed_limit_topic
    return {'plugins': _BASE_GLOBAL_PLUGINS + ['speed_filter'], 'speed_filter': speed_filter}


def build_overrides(
    nav2_params: dict, profile: str, filter_choice: str, odom_topic: str, speed_limit_topic: str = '',
) -> dict:
    """로봇 하나의 controller_server(FollowPath=DWA)·global_costmap·local_costmap에 덧붙일 오버레이 전체.

    nav2.launch.py가 {ns: 이 dict}로 감싸 임시 yaml에 써서, 로봇별 RewrittenYaml 뒤에 추가
    parameters 소스로 얹는다. 기본(warehouse, none)은 지금 controller_server override와 똑같고
    global_costmap·local_costmap 쪽은 빈 dict(변경 없음)다.
    """
    dwa_config = dict(nav2_params['controller_server']['ros__parameters']['DWA'])
    profile_over = profile_overrides(nav2_params, profile)
    dwa_config.update(profile_over.get('DWA', {}))

    global_over = dict(profile_over.get('global_costmap', {}))
    global_over.update(congestion_filter_overrides(filter_choice, speed_limit_topic))
    local_over = dict(profile_over.get('local_costmap', {}))

    return {
        'controller_server': {'ros__parameters': {
            'controller_plugins': ['FollowPath'],
            'FollowPath': dwa_config,
            'odom_topic': odom_topic,
        }},
        'global_costmap': {'global_costmap': {'ros__parameters': global_over}},
        'local_costmap': {'local_costmap': {'ros__parameters': local_over}},
    }
