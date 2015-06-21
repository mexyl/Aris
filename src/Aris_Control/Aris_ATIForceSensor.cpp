#include "Aris_ATIForceSensor.h"

namespace ElmoMotor{

CATIForceSensor::CATIForceSensor()
{
}

CATIForceSensor::~CATIForceSensor()
{
}

int CATIForceSensor::SetEtherCATPosition(uint16_t position,uint32_t product_code,uint32_t vender_id, uint16_t alias)
{
    // Set the device's position in ethercat network
    m_position=position;
    m_product_code=product_code;
    m_vender_id=vender_id;
    m_alias=alias;

    printf("pos %d, alias %d\n", position, alias);
    // Register ethercat domain for this device
    m_domainAllRegs[0]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x6000, 0x01, &m_offsetFx};
    m_domainAllRegs[1]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x6000, 0x02, &m_offsetFy};
    m_domainAllRegs[2]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x6000, 0x03, &m_offsetFz};
    m_domainAllRegs[3]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x6000, 0x04, &m_offsetMx};
    m_domainAllRegs[4]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x6000, 0x05, &m_offsetMy};
    m_domainAllRegs[5]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x6000, 0x06, &m_offsetMz};
    m_domainAllRegs[6]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x7010, 0x01, &m_offsetCw1};
    m_domainAllRegs[7]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x7010, 0x02, &m_offsetCw2};

    // Register the PDO entries for this device
    m_slavePdoEntries[0] = (ec_pdo_entry_info_t){0x7010, 0x01, 32};   // Control code 1
    m_slavePdoEntries[1] = (ec_pdo_entry_info_t){0x7010, 0x02, 32};   // Control code 2

    m_slavePdoEntries[2] = (ec_pdo_entry_info_t){0x6000, 0x01, 32};   // Fx
    m_slavePdoEntries[3] = (ec_pdo_entry_info_t){0x6000, 0x02, 32};   // Fy
    m_slavePdoEntries[4] = (ec_pdo_entry_info_t){0x6000, 0x03, 32};   // Fz
    m_slavePdoEntries[5] = (ec_pdo_entry_info_t){0x6000, 0x04, 32};   // Mx
    m_slavePdoEntries[6] = (ec_pdo_entry_info_t){0x6000, 0x05, 32};   // My
    m_slavePdoEntries[7] = (ec_pdo_entry_info_t){0x6000, 0x06, 32};   // Mz

    m_slavePdoEntries[8] = (ec_pdo_entry_info_t){0x6010, 0x00, 32};  // Status code
    m_slavePdoEntries[9] = (ec_pdo_entry_info_t){0x6020, 0x00, 32};  // Sample counter

    // Register PDOs for the PDO entries just registered
    m_slaveRxPdos[0] = (ec_pdo_info_t){0x1601, 2, m_slavePdoEntries +  0};
    m_slaveTxPdos[0] = (ec_pdo_info_t){0x1a00, 8, m_slavePdoEntries +  2};

    // Assign sync masters for the PDOs just registered
    m_slaveSyncs[0] = (ec_sync_info_t){0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE};
    m_slaveSyncs[1] = (ec_sync_info_t){1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE};
    m_slaveSyncs[2] = (ec_sync_info_t){2, EC_DIR_OUTPUT, 1, m_slaveRxPdos + 0, EC_WD_ENABLE};
    m_slaveSyncs[3] = (ec_sync_info_t){3, EC_DIR_INPUT,  1, m_slaveTxPdos + 0, EC_WD_DISABLE};
    m_slaveSyncs[4] = (ec_sync_info_t){0xff};


    return 0;
}

int CATIForceSensor::Initialize(ec_master_t **p_master)
{
    m_pMaster=*p_master;
    printf("2\n");

    m_pDomainAll = ecrt_master_create_domain(m_pMaster);
    if (!m_pDomainAll)
    {
        printf("create domain failed!\n");
        return -3;
    }
    printf("3\n");
    printf("v:%x:%x\n",(int)m_vender_id,m_product_code);
    printf("pos: %d: %d\n", m_alias, m_position);

    // Get the slave configuration
    if (!(m_slaveConfigATIForceSensor = ecrt_master_slave_config(
        m_pMaster, m_alias,m_position,m_vender_id,m_product_code)))
    {
        printf("Failed to get slave 0 configuration.\n");
        return -4;
    }
    printf("4\n");

    // Configure the slave's PDOs and sync masters
    if (ecrt_slave_config_pdos(m_slaveConfigATIForceSensor, 4, m_slaveSyncs))
    {
        // handle error
        printf("Error slave config PDOs!\n");
        return -5;
    }
    printf("5\n");

    // Configure the slave's domain
    if (ecrt_domain_reg_pdo_entry_list(m_pDomainAll, m_domainAllRegs))
    {
        printf("PDO entry registration failed!\n");
        return -6;
    }
    printf("6\n");
    printf("OK!\n");

    // Set the initial base zero offset value
    for ( int i = 0; i < 6; i++)
    {
        m_baseValue[i] = 0;
        m_summedValue[i] = 0;
    }

    //ecrt_slave_config_dc(m_slaveConfigATIForceSensor, 0x0300, 1000000, 400000, 0, 0);
    return 0;
}

int CATIForceSensor::Activate()
{
    m_pDomainAllPd = ecrt_domain_data(m_pDomainAll);
    if (!m_pDomainAllPd)
    {
        return -10;
    }
    return 0;
}

int CATIForceSensor::ReadData( CForceInputsData *p_data )
{
    p_data->values[0] = EC_READ_S32(m_pDomainAllPd + m_offsetFx);
    p_data->values[1] = EC_READ_S32(m_pDomainAllPd + m_offsetFy);
    p_data->values[2] = EC_READ_S32(m_pDomainAllPd + m_offsetFz);
    p_data->values[3] = EC_READ_S32(m_pDomainAllPd + m_offsetMx);
    p_data->values[4] = EC_READ_S32(m_pDomainAllPd + m_offsetMy);
    p_data->values[5] = EC_READ_S32(m_pDomainAllPd + m_offsetMz);
    // Zeroing procedure is ongoing
    if (m_zeroingCountLast > 0)
    {
        // sum up the readout data
        for (int i = 0; i < 6; i++)
        {
            m_summedValue[i] += p_data->values[i];
        }
        m_zeroingCountLast--;

        // it's time to end zeroing procedure, calculate the average and update baseValue
        if (m_zeroingCountLast <= 0)
        {
            for (int i = 0; i < 6; i++)
            {
                m_baseValue[i] = m_summedValue[i] / ZEROING_DATA_COUNT;
            }
        }
    }
    return 0;
}

void CATIForceSensor::Upload()
{
    ecrt_domain_process(m_pDomainAll);
    ReadData(&m_InputsData);
}

void CATIForceSensor::Download()
{
    EC_WRITE_S32(m_pDomainAllPd + m_offsetCw1,  0);
    EC_WRITE_S32(m_pDomainAllPd + m_offsetCw2,  0);
    ecrt_domain_queue(m_pDomainAll);
}

int CATIForceSensor::GetForceInputs( CForceInputsData* p_data ) const
{
    for (int i = 0; i < 6; i++)
    {
        p_data->values[i] = m_InputsData.values[i] - m_baseValue[i];
    }
    return 0;
}

void CATIForceSensor::RequestZeroing()
{
    m_zeroingCountLast = ZEROING_DATA_COUNT;
    for (int i = 0; i < 6; i++)
    {
        m_summedValue[i] = 0;
    }
}

}
